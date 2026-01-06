package org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GeneralPose {
    private double timeoutMS;
    public GeneralPose() {
        timeoutMS = 0;
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

    public boolean runActions() {
        return false;
    }

    public void setTimeoue(double timeout) {
        timeoutMS = timeout;
    }
    public double getTimeout() {
        return timeoutMS;
    }
}