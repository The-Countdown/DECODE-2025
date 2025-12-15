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
            if (robotContainer.latitudePID.calculate() > Constants.Control.ZERO_POWER_TOLERANCE || robotContainer.longitudePID.calculate() > Constants.Control.ZERO_POWER_TOLERANCE || robotContainer.headingPID.calculate() < Constants.Control.ZERO_POWER_TOLERANCE) {
                robotContainer.drivetrain.powerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.longitudePID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.headingPID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER)
                );
            }
            Thread.yield();
        }
    }

    public void stopThread() {
        enabled = false;
    }
}
