package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.LocalizationUpdater;

import java.util.ArrayList;
import java.lang.Thread;

public class PathPlanner {
    Telemetry telemetry;
    RobotContainer robot;
    ArrayList<Pose2D> poses = new ArrayList<Pose2D>();

    public PathPlanner(Telemetry telemetry, RobotContainer robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void addPose(Pose2D pose) {
        poses.add(pose);
    }

    /**
    * Calculates angle of the target relative to the current position of the robot and drives to target
    * @param index
    */
    public void driveToPose(int index) {
        Pose2D currentPose = LocalizationUpdater.currentPose;
        Status.targetPose = poses.get(index);

        double deltaX = Status.targetPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double deltaY = Status.targetPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM);

        double angleToTarget = Math.atan2(deltaY, deltaX);
        double[] angles = {angleToTarget, angleToTarget, angleToTarget, angleToTarget};
        double[] powers = {0.5,0.5,0.5,0.5};

        robot.drivetrain.swerveSetTargets(angles,powers);
        robot.pathPlanner.waitForTarget();
        robot.drivetrain.swerveSetTargets(angles, Constants.SWERVE_NO_POWER);
    }

    public void driveThroughPath () {
        for (int i = 0; i < poses.size(); i++) {
            driveToPose(i);
        }
        robot.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
    }

    public void waitForTarget() {
        while (!robot.poseMath.isAtPos()){
            Thread.yield();
        }
    }

    public void displayPoses() {
        for (int i = 0; i < poses.size(); i++) {
            telemetry.addData("Pose", poses.get(i).toString());
            telemetry.update();
        }
    }
}
