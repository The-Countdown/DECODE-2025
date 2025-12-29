package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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