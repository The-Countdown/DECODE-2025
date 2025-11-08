package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

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
                    robotContainer.latitudePID.calculate(),
                    robotContainer.longitudePID.calculate(),
                    robotContainer.headingPID.calculate()
            );
            Thread.yield();
        }
    }

    public void stopThread() {
        enabled = false;
    }
}
