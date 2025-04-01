package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointUpdater extends Thread {
    private final RobotManager robotManager;
    public static Pose2D currentPose = new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES,0);
    public static double currentHeading = 0;

    /**
     * The pinpoint takes more time than a normal device, however, I don't know how much that is,
     * so I put it into a thread to be safe
     */
    public PinpointUpdater(RobotManager robotManager) {
        this.robotManager = robotManager;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("PinpointUpdater");
    }

    @Override
    public void run() {
        currentPose = RobotManager.HardwareDevices.pinpoint.getPosition();
        currentHeading = currentPose.getHeading(AngleUnit.DEGREES); // TODO: Invert if needed (could be increasing when going counter-clockwise)
    }
}
