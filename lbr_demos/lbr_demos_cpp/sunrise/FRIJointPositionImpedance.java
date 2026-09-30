/*
 * Sunrise-side POSITION FRI application for streamed joint-position references.
 * Copy into the Sunrise application's application package and set REMOTE_HOST.
 *
 * The PC client must stream measured joint positions during FRI's command-wait
 * state, then continuously stream bounded joint-position references while the
 * session is commanding. It owns trajectory shaping and stale-reference
 * handling. This application intentionally starts from a position hold at the
 * measured configuration and uses the installed Sunrise tool/load definition.
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
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class FRIJointPositionImpedance extends RoboticsAPIApplication {
    private static final String REMOTE_HOST = "192.170.10.1"; // PC running the FRI client
    private static final int SEND_PERIOD_MS = 1;             // 1 ms FRI cycle
    private static final int FRI_RECEIVE_MULTIPLIER = 1;
    private static final double[] JOINT_STIFFNESS_NM_PER_RAD = {
        50.0, 50.0, 40.0, 25.0, 12.5, 12.5, 7.5
    };
    private static final double JOINT_DAMPING = 0.7;

    @Inject private LBR lbr;

    private volatile boolean stopping;
    private FRISession session;
    private IMotionContainer motion;
    private Tool gripper;
    private IUserKeyBar stopKeyBar;

    @Override public void initialize() {
        stopKeyBar = getApplicationUI().createUserKeyBar("FRI Joint Position Control");
        IUserKey stopKey = stopKeyBar.addUserKey(0, new IUserKeyListener() {
            @Override public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                if (event == UserKeyEvent.KeyDown) stopping = true;
            }
        }, true);
        stopKey.setText(UserKeyAlignment.Middle, "STOP");
        stopKeyBar.publish();

        gripper = getApplicationData().createFromTemplate("Gripper");
        gripper.attachTo(lbr.getFlange());
        getLogger().info("Attached Sunrise tool template 'Gripper' to the robot flange.");

        FRIConfiguration configuration = FRIConfiguration.createRemoteConfiguration(lbr, REMOTE_HOST);
        configuration.setSendPeriodMilliSec(SEND_PERIOD_MS);
        configuration.setReceiveMultiplier(FRI_RECEIVE_MULTIPLIER);
        session = new FRISession(configuration);
    }

    @Override public void run() {
        try {
            getLogger().info("Waiting for FRI client at " + REMOTE_HOST);
            session.await(60, TimeUnit.SECONDS);
            if (stopping) return;

            FRIJointOverlay positionOverlay =
                new FRIJointOverlay(session, ClientCommandMode.POSITION);
            JointImpedanceControlMode jointImpedance = new JointImpedanceControlMode(
                JOINT_STIFFNESS_NM_PER_RAD[0], JOINT_STIFFNESS_NM_PER_RAD[1],
                JOINT_STIFFNESS_NM_PER_RAD[2], JOINT_STIFFNESS_NM_PER_RAD[3],
                JOINT_STIFFNESS_NM_PER_RAD[4], JOINT_STIFFNESS_NM_PER_RAD[5],
                JOINT_STIFFNESS_NM_PER_RAD[6]);
            jointImpedance.setDampingForAllJoints(JOINT_DAMPING);

            getLogger().info("Starting joint-impedance position hold with FRI POSITION overlay. "
                + "Verify the configured Sunrise tool and load before enabling the PC client.");
            motion = lbr.moveAsync(positionHold(jointImpedance, -1, null)
                .addMotionOverlay(positionOverlay));

            while (!stopping && !Thread.currentThread().isInterrupted()) {
                if (motion.isFinished()) {
                    getLogger().error("FRI position-hold motion ended; stopping application.");
                    break;
                }
                Thread.sleep(10);
            }
        } catch (TimeoutException ex) {
            getLogger().error("FRI client connection timed out: " + ex.getMessage());
        } catch (CommandInvalidException ex) {
            getLogger().error("FRI position motion was rejected or ended: " + ex.getMessage());
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (Exception ex) {
            getLogger().error("FRI joint-position application failed: " + ex.getMessage());
        } finally {
            cancelAndWait();
            if (session != null) {
                try { session.close(); }
                catch (Exception ex) { getLogger().warn("Could not close FRI session: " + ex.getMessage()); }
            }
        }
    }

    private void cancelAndWait() {
        IMotionContainer activeMotion = motion;
        if (activeMotion == null || activeMotion.isFinished()) return;
        activeMotion.cancel();
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
        while (!activeMotion.isFinished() && System.nanoTime() < deadline) {
            try { Thread.sleep(10); }
            catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        if (!activeMotion.isFinished()) {
            getLogger().error("Timed out waiting for FRI motion cancellation.");
        }
    }
}
