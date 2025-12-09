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
    public int currentPose;

    public PathPlanner(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.currentPose = 0;
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
