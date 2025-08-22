package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseMath {
    private Pose2D currentPose;

    public PoseMath(Pose2D currentPose) {
        this.currentPose = currentPose;
    }

    public void updatePose(Pose2D currentPose) {
        this.currentPose = currentPose;
    }

    public double distanceTo(Pose2D target) {
        double dx = target.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double dy = target.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM);
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Finds the heading error between the current heading and the target heading.
     * @param targetHeading
     * @return The heading error in degrees
     */
    public double HeadingError(double targetHeading) {
        return normalizeAngle(targetHeading - currentPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Finds the angle needed to face the target position
     * @param target
     * @return The angle needed to face the target position in degrees
     */
    public double targetPosDirectionDegrees (Pose2D target) {
        double dx = target.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double dy = target.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM);
        return normalizeAngle(Math.atan2(dy, dx));
    }

    //gives pose between two poses, so the middle of the path
    //bit confused do later
    //do i even need this?
    /*
    public double middlePath(Pose2d other, double t) {

    }
    */

    private double normalizeAngle(double angle) {
        // On this edge case the servo will not move. If fixing the problem is less expensive than this, please do so.
        if (angle == 90) {
            angle = 89.999;
        }
        if (angle == -90 || angle == -180) {
            angle += 0.001;
        }

        // Check if the angle is already in the desired range.
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [-360, 360).
        double normalizedAngle = angle % 360;

        // If the result was negative, shift it to the range [0, 360).
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }

        // If the angle is in the range [180, 360), shift it to [-180, 0).
        if (normalizedAngle >= 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }
}