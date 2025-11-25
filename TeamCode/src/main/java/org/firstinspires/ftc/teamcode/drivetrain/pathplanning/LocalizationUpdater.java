package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class LocalizationUpdater extends Thread {
    private final RobotContainer robotContainer;
    public double CURRENT_LOOP_TIME_MS = 0;
    public double CURRENT_LOOP_TIME_AVG_MS = 0;

    public boolean enabled = true;

    /**
     * The pinpoint takes more time than a normal device, however, I don't know how much that is,
     * so I put it into a thread to be safe.
     */
    public LocalizationUpdater(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("PinpointUpdater");
    }

    @Override
    public void run() {
        if (!enabled) {
            return;
        }
        while (Status.opModeIsActive) {
            RobotContainer.HardwareDevices.pinpoint.update();
//            Status.currentPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
            Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
            CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("pinpointUpdater");
            CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("pinpointUpdater");
            if (Status.isDrivingActive) {
                try {
                    Thread.sleep(Constants.Pathing.PINPOINT_UPDATE_DELAY_MS);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            } else {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        Thread.yield();
    }

    public void stopThread() {
        enabled = false;
    }
}
