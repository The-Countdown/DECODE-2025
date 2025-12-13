package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.main.Constants;
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
        while (Status.opModeIsActive) {
            if (!enabled) {
                return;
            }
            if (Status.isDrivingActive) {
                Thread.yield();
            }
            double max = 0.8;
            if (robotContainer.latitudePID.calculate() > Constants.Control.ZERO_POWER_TOLERANCE || robotContainer.longitudePID.calculate() > Constants.Control.ZERO_POWER_TOLERANCE || robotContainer.headingPID.calculate() < Constants.Control.ZERO_POWER_TOLERANCE) {
                robotContainer.drivetrain.powerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.calculate(), -max, max),
                        HelperFunctions.clamp(robotContainer.longitudePID.calculate(), -max, max),
                        HelperFunctions.clamp(robotContainer.headingPID.calculate(), -max, max)
                );
            }
            Thread.yield();
        }
    }

    public void stopThread() {
        enabled = false;
    }
}
