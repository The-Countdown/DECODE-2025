package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Status;

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
    public boolean pathCompleted = false;
    public int currentPath;

    public PathPlanner(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.currentPath = 0;
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

    public boolean pathTimeOut(Pose2D pose1, Pose2D pose2, ElapsedTime pathTimer){
        double distance = Math.sqrt(Math.pow(pose1.getX(DistanceUnit.CM) - pose2.getX(DistanceUnit.CM), 2) + Math.pow(pose1.getY(DistanceUnit.CM) - pose2.getY(DistanceUnit.CM), 2));
        double timeToComplete = distance-Constants.Pathing.MIN_DISTANCE_FOR_MAX_SPEED_CM/Constants.Pathing.MAX_AUTO_SWERVE_VELOCITY + distance/Constants.Pathing.MAX_SLOWING_CURVE_TIME_MS + Constants.Pathing.PATH_TIMEOUT_MS;
        if (pathTimer.milliseconds() < timeToComplete){
            return true;
        }
        return false;
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
