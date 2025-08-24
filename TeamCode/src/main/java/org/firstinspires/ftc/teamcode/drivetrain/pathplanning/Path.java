package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Path {
    public Pose2D[] points;
    public int currentPoint;
    //accuracy should be a variable
    public Pose2D getPos(Pose2D[] points, int currentPoint) {
        return points[currentPoint];
    }
}