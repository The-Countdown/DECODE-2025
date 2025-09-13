package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

public class HelperFunctions {

    /**
     * @param pose Pose3d to convert to Pose2d
     * @return Pose2d
     */
    public static Pose2D to2D(Pose3D pose) {
        return new Pose2D(DistanceUnit.CM, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    /**
     * Normalizes an angle to the range [-180, 180).
     * <p>
     * (This is duplicated from {@link Drivetrain} because I wanted the constants class to be
     * isolated from the rest of the codebase)
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    public static double normalizeAngle(double angle) {
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
