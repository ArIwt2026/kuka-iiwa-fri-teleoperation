/*
 * Sunrise-side owner for the PC Cartesian-impedance + joint-wall controller.
 * Copy into the Sunrise application's application package and set REMOTE_HOST.
 * This source is not compiled in the ROS workspace.
 */
package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class FRICartesianWall extends RoboticsAPIApplication {
    private static final String REMOTE_HOST = "192.170.10.1"; // PC running the FRI client
    // FRI TORQUE overlays require joint impedance and a send period of 5 ms or less.
    private static final int SEND_PERIOD_MS = 2;
    private static final long POLL_MS = 20;
    private static final long EDGE_LOCKOUT_NS = 1000000000L;
    private static final double WALL_INSET = Math.toRadians(10.0);
    private static final double STARTUP_CLEARANCE = Math.toRadians(1.0);

    @Inject private LBR lbr;
    @Inject private MediaFlangeIOGroup mediaFlange;

    private volatile boolean stopping;
    private volatile IMotionContainer motion;
    private volatile IMotionContainer holdMotion;
    private FRISession session;
    private FRIJointOverlay overlay;
    private boolean previousOutput;
    private long lastAcceptedEdge;
    private IUserKeyBar stopBar;

    @Override public void initialize() {
        stopBar = getApplicationUI().createUserKeyBar("FRI Cartesian Wall");
        IUserKey key = stopBar.addUserKey(0, new IUserKeyListener() {
            @Override public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                if (event == UserKeyEvent.KeyDown) stopping = true;
            }
        }, true);
        key.setText(UserKeyAlignment.Middle, "STOP");
        stopBar.publish();

        FRIConfiguration configuration = FRIConfiguration.createRemoteConfiguration(lbr, REMOTE_HOST);
        configuration.setSendPeriodMilliSec(SEND_PERIOD_MS);
        configuration.setReceiveMultiplier(1);
        configuration.registerIO(mediaFlange.getOutput("Output1"));
        session = new FRISession(configuration);
        overlay = new FRIJointOverlay(session, ClientCommandMode.WRENCH);
    }

    @Override public void run() {
        try {
            getLogger().info("Waiting for FRI client at " + REMOTE_HOST);
            session.await(60, TimeUnit.SECONDS);
            previousOutput = mediaFlange.getOutput1();
            holdStiffPosition();
            getLogger().info("FRI MONITORING/IDLE; Output1 baseline=" + previousOutput
                + ". Robot locked in stiff PositionHold.");

            while (!stopping && !Thread.currentThread().isInterrupted()) {
                IMotionContainer current = motion;
                if (current != null && current.isFinished()) {
                    synchronized (this) {
                        if (motion == current) motion = null;
                    }
                    holdStiffPosition();
                    previousOutput = mediaFlange.getOutput1();
                    getLogger().error("FRI overlay ended unexpectedly; locked in stiff hold until a new Output1 edge.");
                }
                if (observedEdge()) {
                    if (motion != null) {
                        // STOP is never debounced: an accepted start must always be reversible.
                        stopOverlayAndConfirm(true);
                    } else if (startLockoutPassed()) {
                        try {
                            startOverlay();
                        } catch (Exception ex) {
                            getLogger().error("FRI start failed: " + ex.getMessage()
                                + "; remaining in stiff PositionHold.");
                            holdStiffPosition();
                        }
                    }
                }
                Thread.sleep(POLL_MS);
            }
        } catch (TimeoutException ex) {
            getLogger().error("FRI connection timeout: " + ex);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (Exception ex) {
            getLogger().error("FRI Cartesian-wall application stopped: " + ex);
            for (StackTraceElement frame : ex.getStackTrace())
                getLogger().error("at " + frame.toString());
        } finally {
            stopping = true;
            try { stopOverlayAndConfirm(false); }
            catch (Exception ex) { getLogger().error("Final motion stop failed: " + ex); }
            if (session != null) session.close();
        }
    }

    /*
     * High stiffness mode (stiff hold)
     */
    private static final double HIGH_TRANSLATION = 2500;
    private static final double HIGH_ROTATION = 300;
    private static final double HIGH_DAMPING = 0.7;

    /*
     * Low / free mode (teleop hand guiding)
     */
    private static final double LOW_STIFFNESS = 0;
    private static final double LOW_DAMPING = 0.2;

    private void holdStiffPosition() {
        if (stopping) return;
        IMotionContainer current = holdMotion;
        if (current == null || current.isFinished()) {
            CartesianImpedanceControlMode highMode = new CartesianImpedanceControlMode();
            highMode.parametrize(CartDOF.TRANSL).setStiffness(HIGH_TRANSLATION);
            highMode.parametrize(CartDOF.ROT).setStiffness(HIGH_ROTATION);
            highMode.parametrize(CartDOF.ALL).setDamping(HIGH_DAMPING);
            holdMotion = lbr.moveAsync(positionHold(highMode, -1, null));
            getLogger().info("Stiff Cartesian PositionHold active; robot locked in position.");
        }
    }

    private void cancelHoldMotion() throws InterruptedException {
        IMotionContainer hold = holdMotion;
        if (hold == null) return;
        if (!hold.isFinished()) hold.cancel();
        long deadline = System.nanoTime() + 5000000000L;
        while (!hold.isFinished()) {
            if (System.nanoTime() >= deadline)
                throw new IllegalStateException("Hold motion cancellation not confirmed within 5 s");
            Thread.sleep(POLL_MS);
        }
        holdMotion = null;
    }

    private boolean startLockoutPassed() {
        long now = System.nanoTime();
        if (lastAcceptedEdge != 0 && now - lastAcceptedEdge < EDGE_LOCKOUT_NS) {
            getLogger().warn("Output1 START edge ignored during one-second lockout; not queued.");
            return false;
        }
        lastAcceptedEdge = now;
        return true;
    }

    private boolean observedEdge() {
        boolean current = mediaFlange.getOutput1();
        if (current == previousOutput) return false;
        previousOutput = current;
        return true;
    }

    private void startOverlay() throws Exception {
        if (!com.kuka.connectivity.motionModel.smartServo.ServoMotion
                .validateForImpedanceMode(lbr))
            throw new IllegalStateException("Impedance/load validation failed");

        double[] lower = lbr.getJointLimits().getMinJointPosition().get();
        double[] upper = lbr.getJointLimits().getMaxJointPosition().get();
        double[] previous = lbr.getCurrentJointPosition().get();
        checkStartupInsideFreeRegion(previous, lower, upper);
        long previousTime = System.nanoTime();

        // Validate stability over 25 samples (500 ms) WHILE holdMotion keeps the robot firmly locked
        for (int sample = 0; sample < 25; ++sample) {
            Thread.sleep(POLL_MS);
            if (stopping) return;
            // A stop edge during STARTING cancels this start instead of being discarded.
            if (observedEdge()) {
                getLogger().warn("STARTING cancelled by Output1 edge.");
                return;
            }
            double[] current = lbr.getCurrentJointPosition().get();
            checkStartupInsideFreeRegion(current, lower, upper);
            long now = System.nanoTime();
            double dt = (now - previousTime) * 1e-9;
            if (!(dt > 0.0 && dt <= 0.100))
                throw new IllegalStateException("Startup sampling timeout");
            for (int i = 0; i < 7; ++i)
                if (Math.abs(current[i] - previous[i]) / dt > Math.toRadians(0.5))
                    throw new IllegalStateException("Robot moving before FRI start: A" + (i + 1)
                        + " speed_deg_s=" + Math.toDegrees(Math.abs(current[i] - previous[i]) / dt)
                        + " allowed_deg_s=0.5 dt_ms=" + dt * 1000.0);
            previous = current;
            previousTime = now;
        }

        double[] finalPosition = lbr.getCurrentJointPosition().get();
        checkStartupInsideFreeRegion(finalPosition, lower, upper);
        for (int i = 0; i < 7; ++i)
            if (Math.abs(finalPosition[i] - previous[i]) > Math.toRadians(0.05))
                throw new IllegalStateException("Robot moved immediately before FRI start: A" + (i + 1)
                    + " delta_deg=" + Math.toDegrees(Math.abs(finalPosition[i] - previous[i]))
                    + " allowed_deg=0.05");

        // Stability confirmed while held; now transition seamlessly to compliant overlay motion
        cancelHoldMotion();

        CartesianImpedanceControlMode freeMode = new CartesianImpedanceControlMode();
        freeMode.parametrize(CartDOF.ALL).setStiffness(LOW_STIFFNESS);
        freeMode.parametrize(CartDOF.ALL).setDamping(LOW_DAMPING);
        freeMode.setNullSpaceStiffness(0.0);
        freeMode.setNullSpaceDamping(0.2);
        synchronized (this) {
            if (stopping || motion != null) return;
            motion = lbr.moveAsync(positionHold(freeMode, -1, null).addMotionOverlay(overlay));
        }
        getLogger().info("FRI WRENCH overlay STARTED (Cartesian free mode with A4 soft wall); PC controller owns overlay wrench.");
    }

    private void checkStartupInsideFreeRegion(double[] q, double[] lower, double[] upper) {
        for (int i = 0; i < 7; ++i) {
            if (Double.isNaN(q[i]) || Double.isInfinite(q[i])) {
                throw new IllegalStateException("Non-finite joint position at A" + (i + 1));
            }
        }
        double requiredMargin = WALL_INSET + STARTUP_CLEARANCE;
        int i = 3;
        if (q[i] <= lower[i] + requiredMargin || q[i] >= upper[i] - requiredMargin) {
            throw new IllegalStateException("A4 starts inside joint-wall region; required distance from limit is > "
                + Math.toDegrees(requiredMargin) + " deg");
        }
    }

    private void stopOverlayAndConfirm(boolean returnToIdleHold) throws InterruptedException {
        IMotionContainer current = motion;
        if (current != null) {
            if (!current.isFinished()) current.cancel();
            long deadline = System.nanoTime() + 5000000000L;
            while (!current.isFinished()) {
                if (System.nanoTime() >= deadline)
                    throw new IllegalStateException("Motion stop not confirmed within 5 s");
                Thread.sleep(POLL_MS);
            }
            motion = null;
        }
        if (returnToIdleHold) {
            holdStiffPosition();
        } else {
            cancelHoldMotion();
        }
        // Rebase so edges occurring during cleanup do not become delayed starts.
        previousOutput = mediaFlange.getOutput1();
        getLogger().info(returnToIdleHold
            ? "FRI torque overlay STOPPED; robot locked in stiff PositionHold; back in monitoring/idle."
            : "FRI torque overlay STOPPED.");
    }

    @Override public void dispose() {
        stopping = true;
        IMotionContainer current = motion;
        if (current != null) {
            try { if (!current.isFinished()) current.cancel(); }
            catch (Exception ex) { getLogger().error("Motion cancellation failed: " + ex); }
        }
        IMotionContainer hold = holdMotion;
        if (hold != null) {
            try { if (!hold.isFinished()) hold.cancel(); }
            catch (Exception ex) { getLogger().error("Hold motion cancellation failed: " + ex); }
        }
        if (session != null) session.close();
        super.dispose();
    }
}
