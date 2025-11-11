package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class PathingUpdater extends Thread {
    private final RobotContainer robotContainer;
    private boolean enabled = true;

    public PathingUpdater(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        setDaemon(true);
        setName("PathingUpdater");
    }

    @Override
    public void run() {
        if (!enabled) {
            return;
        }
        while (Status.opModeIsActive) {
            robotContainer.drivetrain.powerInput(
                    HelperFunctions.clamp(robotContainer.latitudePID.calculate(), -0.6, 0.6),
                    HelperFunctions.clamp(robotContainer.longitudePID.calculate(), -0.6, 0.6),
                    HelperFunctions.clamp(robotContainer.headingPID.calculate(), -0.6, 0.6)
            );
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void stopThread() {
        enabled = false;
    }
}
