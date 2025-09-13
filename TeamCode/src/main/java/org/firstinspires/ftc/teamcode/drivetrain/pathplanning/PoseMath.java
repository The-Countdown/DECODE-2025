package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.LocalizationUpdater;

public class PoseMath {
    public double distanceTo() {
        double dx = Status.targetPose.getX(DistanceUnit.CM) - LocalizationUpdater.currentPose.getX(DistanceUnit.CM);
        double dy = Status.targetPose.getY(DistanceUnit.CM) - LocalizationUpdater.currentPose.getY(DistanceUnit.CM);
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Finds the heading error between the current heading and the target heading.
     * @param targetHeading The target heading in degrees
     * @return The heading error in degrees
     */
    public double headingError(double targetHeading) {
        return normalizeAngle(targetHeading - LocalizationUpdater.currentPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Finds the angle needed to face the target position
     * @param target The target position
     * @return The angle needed to face the target position in degrees
     */
    public double targetPosDirectionDegrees (Pose2D target) {
        double dx = target.getX(DistanceUnit.CM) - LocalizationUpdater.currentPose.getX(DistanceUnit.CM);
        double dy = target.getY(DistanceUnit.CM) - LocalizationUpdater.currentPose.getY(DistanceUnit.CM);
        return normalizeAngle(Math.atan2(dy, dx));
    }

    /**
     * Calculates when x and y distance is <= PATHING_ERROR_MARGIN_CM
     * @return True when robot gets within PATHING_ERROR_MARGIN_CM
     */
    public boolean isAtPos() {
        return Status.targetPose.getX(DistanceUnit.CM) - LocalizationUpdater.currentPose.getX(DistanceUnit.CM) <= Constants.PATHING_ERROR_MARGIN_CM &&
                Status.targetPose.getY(DistanceUnit.CM) - LocalizationUpdater.currentPose.getY(DistanceUnit.CM) <= Constants.PATHING_ERROR_MARGIN_CM;
    }

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
