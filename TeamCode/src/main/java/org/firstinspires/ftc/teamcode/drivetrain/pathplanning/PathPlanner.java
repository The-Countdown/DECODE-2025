package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager.Action;

import java.util.ArrayList;

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
//There are a few more I need to write the rest.

public class PathPlanner {
    private Telemetry telemetry;
    ArrayList<GeneralPose> poses = new ArrayList<>();
    public int pointAmount = 0;
    private final RobotContainer robotContainer;
    public boolean pathCompleted = false;
    public int currentPath;
    public double[] estimatedPathTimes;
    ArrayList<Double> powers = new ArrayList<>();
    ArrayList<Double> speeds = new ArrayList<>();
    ArrayList<Double> times = new ArrayList<>();
    double maxAccelerationDistance;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.currentPath = 0;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(new PositionPose(pose));
        pointAmount += 1;
    }

    public void addPose(double time) {
        poses.add(new SleepPose(time));
        pointAmount += 1;
    }

    public boolean driveUsingPID(int index) {
        if (poses.get(index) instanceof PositionPose) {
            Status.targetPose = poses.get(index).getPose();
        } else if (poses.get(index) instanceof SleepPose) {
            return poses.get(index).getDone();
        }
        return PoseMath.isAtPos();
    }

    public void driveThroughPath(ElapsedTime pathTimer) {
        if (!this.pathCompleted) {
            if (driveUsingPID(this.currentPath) || pathTimeOut(pathTimer)) {
                this.currentPath += 1;
                if (this.currentPath == this.poses.size()) {
                    this.pathCompleted = true;
                }
            }
        }
    }

    public boolean pathTimeOut(ElapsedTime pathTimer){
        return pathTimer.milliseconds() > estimatedPathTimes[currentPath] + Constants.Pathing.PATH_TIMEOUT_ERROR_MS;
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

        public double getSleepTime() {
            return 0;
        }

        public void runActions() {
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
        public double getSleepTime() {
            return this.sleepTime;
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

    public class ActionPose extends GeneralPose {
        private Action[] delayedActions;

        public ActionPose(Action... actions) {
            this.delayedActions = actions;
        }

        @Override
        public void runActions() {
            for (int i = 0; i < this.delayedActions.length; i++) {
                this.delayedActions[i].execute();
            }
        }
    }
}
