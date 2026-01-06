package org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionPose extends GeneralPose {
    private Pose2D pose;

    public PositionPose(Pose2D pose) {
        this.pose = pose;
    }

    public PositionPose(Pose2D pose, double timeoutMS) {
        this.pose = pose;
        setTimeout(timeoutMS);
    }

    @Override
    public Pose2D getPose() {
        return this.pose;
    }
}