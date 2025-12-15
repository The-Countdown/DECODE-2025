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
    private final RobotContainer robotContainer;
    public boolean pathCompleted = false;
    public int currentPath;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.currentPath = 0;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(new PositionPose(pose));
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

    public boolean pathTimeOut(ElapsedTime pathTimer){
        if (pathTimer.milliseconds() < getEstimatedPathTime()){
            return true;
        }
        return false;
    }

    public double getEstimatedPathTime() {
        ArrayList<Double> powers = new ArrayList<>();
        ArrayList<Double> speeds = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();
        double max = 0.8;

        double xDiff = poses.get(currentPath).getPose().getX(DistanceUnit.CM) - poses.get(currentPath-1).getPose().getX(DistanceUnit.CM);
        double yDiff = poses.get(currentPath).getPose().getY(DistanceUnit.CM) - poses.get(currentPath-1).getPose().getY(DistanceUnit.CM);
        double hDiff = HelperFunctions.normalizeAngle(poses.get(currentPath).getPose().getHeading(AngleUnit.DEGREES) - poses.get(currentPath-1).getPose().getHeading(AngleUnit.DEGREES));
        double totalDiff = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        double xSplit = xDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double ySplit = yDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double hSplit = hDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double splitDist = totalDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double lastXError = 0;
        double lastYError = 0;
        double lastHError = 0;
        double currentTime = 0;

        for (int i = Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME; i > 0; i--){
            powers.add(robotContainer.drivetrain.fakePowerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.fakeCalculate(xDiff, currentTime, lastXError), -max, max),
                        HelperFunctions.clamp(robotContainer.longitudePID.fakeCalculate(yDiff, currentTime, lastYError), -max, max),
                        HelperFunctions.clamp(robotContainer.headingPID.fakeCalculate(hDiff, currentTime, lastHError), -max, max)
            )[1]);

            speeds.add((ticksPerSecondOfMotor(powers.get(i)) / Constants.Swerve.MOTOR_TICKS_PER_REVOLUTION) * Constants.Swerve.MOTOR_TO_WHEEL_GEAR_RATIO * 2 * Math.PI * (Constants.Robot.WHEEL_DIAMETER_MM / 10));
            times.add(speeds.get(i)/splitDist);

            currentTime += times.get(i);
            xDiff -= i*xSplit;
            yDiff -= i*ySplit;
            hDiff -= i*hSplit;

            lastXError = robotContainer.latitudePID.fakeCalculate(xDiff, currentTime, lastXError);
            lastYError = robotContainer.longitudePID.fakeCalculate(yDiff, currentTime, lastYError);
            lastHError = robotContainer.headingPID.fakeCalculate(hDiff, currentTime, lastHError);

            if (xDiff < 0) xDiff = 0;
            if (yDiff < 0) yDiff = 0;
            if (hDiff < 0) hDiff = 0;
        }
        //The total time to complete the path = currentTime;
        return currentTime;
    }

    public double ticksPerSecondOfMotor(double power){
        return Constants.Pathing.MAX_SWERVE_VELOCITY * power / 0.8;
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
