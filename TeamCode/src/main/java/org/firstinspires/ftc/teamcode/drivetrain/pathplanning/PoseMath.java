package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class PoseMath {
    public static double distanceToTarget() {
        double dx = Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
        double dy = Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Finds the heading error between the current heading and the target heading.
     *
     * @param targetHeading The target heading in degrees
     * @return The heading error in degrees
     */
    public static double headingError(double targetHeading) {
        return HelperFunctions.normalizeAngle(targetHeading - Status.currentPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Finds the angle needed to face the target position
     *
     * @param target The target position
     * @return The angle needed to face the target position in degrees
     */
    public static double targetPosDirectionDegrees(Pose2D target) {
        double dx = target.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
        double dy = target.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        return HelperFunctions.normalizeAngle(Math.atan2(dy, dx));
    }

    /**
     * Calculates when x and y distance is >= -PATHING_ERROR_MARGIN_CM
     * OR
     * Calculates when x and y distance is <= PATHING_ERROR_MARGIN_CM
     *
     * @return True when robot gets within PATHING_ERROR_MARGIN_CM
     */
    public static boolean isAtPos() {
        return (
                Status.currentPose.getX(DistanceUnit.CM) + Constants.Pathing.PATHING_ERROR_MARGIN_CM >= Status.targetPose.getX(DistanceUnit.CM) &&
                Status.currentPose.getX(DistanceUnit.CM) - Constants.Pathing.PATHING_ERROR_MARGIN_CM <= Status.targetPose.getX(DistanceUnit.CM)) &&
                Status.currentPose.getY(DistanceUnit.CM) + Constants.Pathing.PATHING_ERROR_MARGIN_CM >= Status.targetPose.getY(DistanceUnit.CM) &&
                Status.currentPose.getY(DistanceUnit.CM) - Constants.Pathing.PATHING_ERROR_MARGIN_CM <= Status.targetPose.getY(DistanceUnit.CM)
        ;
    }
}
