package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;

public class HelperFunctions {

    /**
     * @param pose Pose3d to convert to Pose2d
     * @return Pose2d
     */
    public static Pose2D to2D(Pose3D pose) {
        return new Pose2D(DistanceUnit.CM, pose.getPosition().x * 100, pose.getPosition().y * 100, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES));
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

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double disToGoal() {
            double xDiff = Status.GOAL_POSE.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
            double yDiff = Status.GOAL_POSE.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    // Assuming point 1 is less than point 2
    public static double interpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }

    public static int getRandomWithin(int num, double percent) {
        double adjustment = num * percent;
        return (int) (num + Math.random() > percent ? adjustment : -adjustment);
    }
}
