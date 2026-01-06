package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses.ActionPose;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses.GeneralPose;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses.PositionPose;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses.SleepPose;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

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
//There are a few more I need to write the rest.

public class PathPlanner {
    private Telemetry telemetry;
    ArrayList<GeneralPose> poses = new ArrayList<>();
    public int pointAmount = 0;
    private final RobotContainer robotContainer;
    public boolean pathCompleted = false;
    public int currentPath;
    public ElapsedTime timeoutCheck = new ElapsedTime();

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.currentPath = 0;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(new PositionPose(pose));
        pointAmount += 1;
    }

    public void addPoseTimeout(Pose2D pose, double timeoutMS) {
        poses.add(new PositionPose(pose, timeoutMS));
        pointAmount += 1;
    }

    public void addSleepPose(double time) {
        poses.add(new SleepPose(time));
        pointAmount += 1;
    }

    public void addActionPose(ActionPose actionPose) {
        poses.add(actionPose);
        pointAmount += 1;
    }

    public boolean driveUsingPID(int index) {
        // Skip pose if past timeout
        if (poses.get(index).getTimeout() != 0) {
            if (poses.get(index).getTimeout() < timeoutCheck.milliseconds()) {
                timeoutCheck.reset();
                return true;
            }
        }

        if (poses.get(index) instanceof PositionPose) {
            Status.targetPose = poses.get(index).getPose();
            if (PoseMath.isAtPos()) {
                timeoutCheck.reset();
                return true;
            }
        } else if (poses.get(index) instanceof SleepPose) {
            timeoutCheck.reset();
            return poses.get(index).getDone();
        } else if (poses.get(index) instanceof ActionPose) {
            timeoutCheck.reset();
            return poses.get(index).runActions();
        }
        return false;
    }

    public void driveThroughPath(ElapsedTime pathTimer) {
        if (!this.pathCompleted) {
            if (driveUsingPID(this.currentPath)) {
                this.currentPath += 1;
                if (this.currentPath == this.poses.size()) {
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
        pointAmount = 0;
    }
}
