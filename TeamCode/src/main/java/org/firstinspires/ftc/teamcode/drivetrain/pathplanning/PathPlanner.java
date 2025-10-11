package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.ArrayList;
import java.lang.Thread;

public class PathPlanner {
    private Telemetry telemetry;
    private RobotContainer robot;
    private DelayedActionManager delayedActionManager;
    ArrayList<Pose2D> poses = new ArrayList<>();
    boolean atTarget;

    double tolerance = 1;

    public PathPlanner(Telemetry telemetry, RobotContainer robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void addPose(Pose2D pose) {
        poses.add(pose);
    }

    /**
    * Calculates angle of the target relative to the current position of the robot and drives to target
    * @param index which pose to drive to from first to last
    */
    public void driveToPose(int index) {
        atTarget = false;
        while (!atTarget) {
            Status.targetPose = poses.get(index);
            robot.telemetry.addData("Current Position X: ", Status.currentPose.getX(DistanceUnit.CM));
            robot.telemetry.addData("Current Position Y: ", Status.currentPose.getY(DistanceUnit.CM));
            robot.telemetry.update();

            robot.telemetry.addData("diff X", Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM));
            robot.telemetry.addData("diff Y", Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM));

            // Know when the robot is there
            if (Status.currentPose.getX(DistanceUnit.CM) + tolerance >= Status.targetPose.getX(DistanceUnit.CM) && Status.currentPose.getX(DistanceUnit.CM) - tolerance <= Status.targetPose.getX(DistanceUnit.CM)) { // Check X
                if (Status.currentPose.getY(DistanceUnit.CM) + tolerance >= Status.targetPose.getY(DistanceUnit.CM) && Status.currentPose.getY(DistanceUnit.CM) - tolerance <= Status.targetPose.getY(DistanceUnit.CM)) { // Check Y
                    atTarget = true;
                }
            }

            double deltaX = Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            double deltaY = Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);

            double angleToTarget = -Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90;
            double[] angles = {angleToTarget, angleToTarget, angleToTarget, angleToTarget};

            double[] powers;
            // Slow down if close to the target.
            if (deltaX > 5 || deltaY > 5) {
                powers = new double[]{0.5, 0.5, 0.5, 0.5};
            } else {
                powers = new double[]{0.2, 0.2, 0.2, 0.2};
            }


            robot.drivetrain.swerveSetTargets(angles, powers);
//            robot.pathPlanner.waitForTarget();
        }
        double[] emptyAngles = {0, 0, 0, 0};
        robot.drivetrain.swerveSetTargets(emptyAngles, Constants.SWERVE_NO_POWER);
    }

    public void driveThroughPath () {
        delayedActionManager.run();
        for (int i = 0; i < poses.size(); i++) {
            driveToPose(i);
        }
    }

    public void waitForTarget() {
        while (!PoseMath.isAtPos()){
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
