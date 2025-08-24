package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Marker {
    public Pose2D markerPose;

    public Marker(Pose2D pose) {
        markerPose = pose;
    }

    public Pose2D getPose() {
        return markerPose;
    }
    
    private Pose2D point;
}
