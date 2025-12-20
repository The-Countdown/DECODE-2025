package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class PathingUpdater extends Thread {
    private final RobotContainer robotContainer;
    private boolean enabled = true;
    public ElapsedTime timer = new ElapsedTime();

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
            if (Math.abs(robotContainer.latitudePID.calculate()) > Constants.Control.ZERO_POWER_TOLERANCE || Math.abs(robotContainer.longitudePID.calculate()) > Constants.Control.ZERO_POWER_TOLERANCE || Math.abs(robotContainer.headingPID.calculate()) > Constants.Control.ZERO_POWER_TOLERANCE) {
                robotContainer.drivetrain.powerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.longitudePID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.headingPID.calculate(), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER)
                );
            } else {
                robotContainer.drivetrain.powerInput(0,0,0);
            }
            Thread.yield();
        }
    }

    public void stopThread() {
        enabled = false;
    }
}
