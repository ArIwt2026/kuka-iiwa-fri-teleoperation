/*
 * Sunrise-side standalone joint-impedance hold for checking the robot's
 * built-in gravity compensation. No FRI client or streamed overlay is used.
 * Copy into the Sunrise application's application package.
 */
package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

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

public class JointImpedanceHold extends RoboticsAPIApplication {
    private static final double[] JOINT_STIFFNESS_NM_PER_RAD = {
        50.0, 50.0, 40.0, 25.0, 12.5, 12.5, 7.5
    };
    private static final double JOINT_DAMPING = 0.7;
    private static final long POLL_MS = 10;
    private static final long CANCEL_TIMEOUT_MS = 5000;

    @Inject private LBR lbr;

    private volatile boolean stopping;
    private IMotionContainer motion;
    private Tool gripper;

    @Override public void initialize() {
        IUserKeyBar stopKeyBar = getApplicationUI().createUserKeyBar("Joint Impedance Hold");
        IUserKey stopKey = stopKeyBar.addUserKey(0, new IUserKeyListener() {
            @Override public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                if (event == UserKeyEvent.KeyDown) stopping = true;
            }
        }, true);
        stopKey.setText(UserKeyAlignment.Middle, "STOP");
        stopKeyBar.publish();
    }

    @Override public void run() {
        try {
            gripper = getApplicationData().createFromTemplate("Gripper");
            gripper.attachTo(lbr.getFlange());
            getLogger().info("Attached Sunrise tool template 'Gripper' to the robot flange.");

            JointImpedanceControlMode jointImpedance = new JointImpedanceControlMode(
                JOINT_STIFFNESS_NM_PER_RAD[0], JOINT_STIFFNESS_NM_PER_RAD[1],
                JOINT_STIFFNESS_NM_PER_RAD[2], JOINT_STIFFNESS_NM_PER_RAD[3],
                JOINT_STIFFNESS_NM_PER_RAD[4], JOINT_STIFFNESS_NM_PER_RAD[5],
                JOINT_STIFFNESS_NM_PER_RAD[6]);
            jointImpedance.setDampingForAllJoints(JOINT_DAMPING);

            getLogger().info("Starting joint-impedance PositionHold at the current pose; "
                + "stiffness [Nm/rad]=" + formatGains()
                + ", damping=" + JOINT_DAMPING + ". No FRI client is required.");
            motion = lbr.moveAsync(positionHold(jointImpedance, -1, null));

            while (!stopping && !Thread.currentThread().isInterrupted()) {
                if (motion.isFinished()) {
                    getLogger().error("Joint-impedance hold ended unexpectedly.");
                    break;
                }
                Thread.sleep(POLL_MS);
            }
        } catch (CommandInvalidException ex) {
            getLogger().error("Joint-impedance hold was rejected or ended: " + ex.getMessage());
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (Exception ex) {
            getLogger().error("Joint-impedance hold failed: " + ex.getMessage());
        } finally {
            cancelAndWait();
        }
    }

    private String formatGains() {
        StringBuilder result = new StringBuilder("[");
        for (int i = 0; i < JOINT_STIFFNESS_NM_PER_RAD.length; ++i) {
            if (i > 0) result.append(", ");
            result.append(JOINT_STIFFNESS_NM_PER_RAD[i]);
        }
        return result.append(']').toString();
    }

    private void cancelAndWait() {
        IMotionContainer activeMotion = motion;
        if (activeMotion == null || activeMotion.isFinished()) return;

        activeMotion.cancel();
        long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(CANCEL_TIMEOUT_MS);
        while (!activeMotion.isFinished() && System.nanoTime() < deadline) {
            try {
                Thread.sleep(POLL_MS);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        if (!activeMotion.isFinished()) {
            getLogger().error("Timed out waiting for joint-impedance hold cancellation.");
        }
    }
}
