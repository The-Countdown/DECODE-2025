package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    private RobotContainer robotContainer;
    private DelayedActionManager delayedActionManager;
    ArrayList<Pose2D> poses = new ArrayList<>();
    boolean atTarget;
    double tolerance = 1;
    private static final ElapsedTime stopTimer = new ElapsedTime();
    double[] calculatedAngles = new double[Constants.Swerve.NUM_SERVOS];
    double[] calculatedPowers = new double[Constants.Swerve.NUM_MOTORS];
    double[] lastAngles = Constants.Swerve.STOP_FORMATION;
    public boolean pathCompleted = false;
    public int currentPose = 0;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(pose);
    }

    /**
    * Calculates angle of the target relative to the current position of the robotContainer and drives to target
    * @param index which pose to drive to from first to last
    */
    public void driveToPose(int index) {
        atTarget = false;
        while (!atTarget) {
            Status.targetPose = poses.get(index);
            robotContainer.telemetry.addData("Current Position X: ", Status.currentPose.getX(DistanceUnit.CM));
            robotContainer.telemetry.addData("Current Position Y: ", Status.currentPose.getY(DistanceUnit.CM));

            robotContainer.telemetry.addData("diff X", Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM));
            robotContainer.telemetry.addData("diff Y", Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM));
            robotContainer.telemetry.update();

            // Know when the robotContainer is there
            if (Status.currentPose.getX(DistanceUnit.CM) + tolerance >= Status.targetPose.getX(DistanceUnit.CM) && Status.currentPose.getX(DistanceUnit.CM) - tolerance <= Status.targetPose.getX(DistanceUnit.CM)) { // Check X
                if (Status.currentPose.getY(DistanceUnit.CM) + tolerance >= Status.targetPose.getY(DistanceUnit.CM) && Status.currentPose.getY(DistanceUnit.CM) - tolerance <= Status.targetPose.getY(DistanceUnit.CM)) { // Check Y
                    atTarget = true;
                }
            }

            double deltaX = Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            double deltaY = Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);

//            double angleToTarget = -Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90;
            double angleToTarget = -Math.toDegrees(Math.atan2(deltaY, deltaX));
            double[] angles = {angleToTarget, angleToTarget, angleToTarget, angleToTarget};

            double[] powers;
            // Slow down if close to the target.
            if (deltaX > 5 || deltaY > 5) {
                powers = new double[] {0.1, 0.1, 0.1, 0.1};
            } else {
                powers = new double[] {0.1, 0.1, 0.1, 0.1};
            }

            robotContainer.drivetrain.setTargets(angles, powers);
//            robotContainer.pathPlanner.waitForTarget();
        }
        double[] emptyAngles = {0, 0, 0, 0};
        robotContainer.drivetrain.setTargets(emptyAngles, Constants.Swerve.NO_POWER);
    }

    public boolean driveUsingPID(int index) {
        Status.targetPose = poses.get(index);
        return PoseMath.isAtPos();
    }

    public boolean driveUsingPIDWithoutThread(int index) {
        Status.targetPose = poses.get(index);
        robotContainer.drivetrain.powerInput(
                robotContainer.latitudePID.calculate(),
                robotContainer.longitudePID.calculate(),
                robotContainer.headingPID.calculate()
        );
        return PoseMath.isAtPos();
    }

    public void setTarget(int index) {
        Status.targetPose = poses.get(index);
        Status.currentPath = index;
        Status.pathCompleted[index] = false;
    }

    public void updatePathStatus() {
        if (Status.currentPath == -1) {
            return;
        } else {
            Status.pathCompleted[Status.currentPath] = PoseMath.isAtPos();
        }
    }

    public void driveThroughPath () {
        if (!this.pathCompleted) {
            if (driveUsingPID(this.currentPose)) {
                this.currentPose += 1;
                if (this.currentPose == this.poses.size()) {
                    this.pathCompleted = true;
                }
            }
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
