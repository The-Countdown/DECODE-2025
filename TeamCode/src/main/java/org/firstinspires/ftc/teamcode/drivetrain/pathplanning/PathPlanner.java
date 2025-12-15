package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.ArrayList;
import java.lang.Thread;

// Coordinate graphs for reference. These all assume that you are looking at the zero heading orientation (Looking towards the goal assuming Teleop and Auto were started in the correct orientation).
//
// LimeLight
//      x
//      ^
// -y <- -> y
//      |
//     -x
//
// Teleop Pinpoint
//      y
//      ^
// x <-   -> -x
//      |
//     -y
//
//     There are a few more I need to write the rest.

public class PathPlanner {
    private Telemetry telemetry;
    ArrayList<GeneralPose> poses = new ArrayList<>();
    public double pointAmount = 0;
    private final RobotContainer robotContainer;
    public boolean pathCompleted = false;
    public int currentPath;
    public ArrayList<Double> estimatedPathTimes = new ArrayList<>();
    ArrayList<Double> powers = new ArrayList<>();
    ArrayList<Double> speeds = new ArrayList<>();
    ArrayList<Double> times = new ArrayList<>();
    double maxAccelerationDistance;
    double maxAccelerationTime;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.currentPath = 0;
        this.robotContainer = robotContainer;
        maxAccelerationDistance = Constants.Pathing.ACCELERATION_TABLE.lastKey();
        maxAccelerationTime = Constants.Pathing.ACCELERATION_TABLE.get(maxAccelerationDistance);
    }

    public void addPose(Pose2D pose) {
        poses.add(new PositionPose(pose));
        pointAmount += 1;
    }

    public void addPose(double time) {
        poses.add(new SleepPose(time));
    }

    public boolean driveUsingPID(int index) {
        if (poses.get(index) instanceof PositionPose) {
            Status.targetPose = poses.get(index).getPose();
        } else if (poses.get(index) instanceof SleepPose) {
            return poses.get(index).getDone();
        }
        return PoseMath.isAtPos();
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
            //if (driveUsingPID(this.currentPath) || pathTimeOut(poses.get(this.currentPath-1).getPose(), poses.get(this.currentPath).getPose(), pathTimer)) {
            if (driveUsingPID(this.currentPath)) {
                this.currentPath += 1;
                if (this.currentPath == this.poses.size()) {
                    this.pathCompleted = true;
                }
            }
        }
    }

    public void updatePathTimes() {
        for (int i = 0; i < pointAmount - 1; i++) {
            estimatedPathTimes.set(i, calculateEstimatedPathTime(i));
        }
    }

    public boolean pathTimeOut(ElapsedTime pathTimer){
        return pathTimer.milliseconds() < estimatedPathTimes.get(currentPath);
    }

    public double calculateEstimatedPathTime(int path) {
        Pose2D endingPathPose = poses.get(path).getPose();
        Pose2D startingPathPose = poses.get(path - 1).getPose();

        double xDiff = endingPathPose.getX(DistanceUnit.CM) - startingPathPose.getX(DistanceUnit.CM);
        double yDiff = endingPathPose.getY(DistanceUnit.CM) - startingPathPose.getY(DistanceUnit.CM);
        double hDiff = HelperFunctions.normalizeAngle(endingPathPose.getHeading(AngleUnit.DEGREES) - startingPathPose.getHeading(AngleUnit.DEGREES));
        double totalDiff = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        double xSplit = xDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double ySplit = yDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double hSplit = hDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double splitDist = totalDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;

        double remainingAccelerationDistance = maxAccelerationDistance;

        double lastXError = 0;
        double lastYError = 0;
        double lastHError = 0;
        double currentTime = 0;

        for (int i = Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME; i > 0; i--){
            powers.add(robotContainer.drivetrain.fakePowerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.fakeCalculate(xDiff, currentTime, lastXError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.longitudePID.fakeCalculate(yDiff, currentTime, lastYError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.headingPID.fakeCalculate(hDiff, currentTime, lastHError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER)
            )[1]);

            if (remainingAccelerationDistance > maxAccelerationDistance - (maxAccelerationDistance * powers.get(i))) {
                currentTime += accelerationTableInterpolation(splitDist);
            } else {
                //cm per second
                speeds.add((ticksPerSecondOfMotor(powers.get(i)) / Constants.Swerve.MOTOR_TICKS_PER_REVOLUTION) * Constants.Swerve.MOTOR_TO_WHEEL_GEAR_RATIO * 2 * Math.PI * (Constants.Robot.WHEEL_DIAMETER_MM / 10));
                times.add(speeds.get(i)/splitDist);
                currentTime += times.get(i);
            }

            lastXError = xDiff;
            lastYError = yDiff;
            lastHError = hDiff;

            xDiff -= xSplit;
            yDiff -= ySplit;
            hDiff -= hSplit;

            remainingAccelerationDistance -= Math.sqrt(Math.pow(xSplit, 2) + Math.pow(ySplit, 2));

            if (xDiff <= 0.1) break;
            if (yDiff <= 0.1) break;
            if (hDiff <= 0.1) break;
        }
        //The total time to complete the path = currentTime;
        return currentTime;
    }

    public double ticksPerSecondOfMotor(double power){
        //Should be clamped between -0.8 and 0.8, not division, like this? You seem to not even need to clamp it as it is already done on line 116
        return Constants.Pathing.SWERVE_MAX_VELOCITY * HelperFunctions.clamp(power, -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER);
//        return Constants.Pathing.MAX_SWERVE_VELOCITY * power / 0.8;
    }

    public double accelerationTableInterpolation(double distance) {
        distance = Math.abs(distance);
        double distanceLower;
        double distanceUpper;

        if (Constants.Pathing.ACCELERATION_TABLE.floorKey(distance) == null) {
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance);
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.higherKey(distanceLower);
        } else if (Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance) == null) {
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.floorKey(distance);
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.lowerKey(distanceUpper);
        } else {
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.floorKey(distance);
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance);
        }

        return HelperFunctions.interpolate(
                Constants.Pathing.ACCELERATION_TABLE.get(distanceLower),
                Constants.Pathing.ACCELERATION_TABLE.get(distanceUpper),
                (distance - distanceLower) / (distanceUpper - distanceLower)
                );
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

    public void clearPoses() {
        this.poses.clear();
        pointAmount = 0;
    }

    public class GeneralPose {
        public GeneralPose() {
        }

        public Pose2D getPose() {
            return null;
        }

        public boolean getDone() {
            return false;
        }
    }

    public class PositionPose extends GeneralPose {
        private Pose2D pose;

        public PositionPose(Pose2D pose) {
            this.pose = pose;
        }

        @Override
        public Pose2D getPose() {
            return this.pose;
        }
    }

    public class SleepPose extends GeneralPose {
        public double sleepTime; // In miliseconds
        public ElapsedTime sleepTimer = null;
        public SleepPose(double time) {
            this.sleepTime = time;
        }

        @Override
        public boolean getDone() {
            if (this.sleepTimer == null) {
                this.sleepTimer = new ElapsedTime();
                this.sleepTimer.reset();
            }
            if (this.sleepTimer.milliseconds() > this.sleepTime) {
                this.sleepTimer = null;
                return true;
            }
            return false;
        }
    }
}
