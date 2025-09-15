package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * The `Status` class serves as a centralized repository for tracking the
 * operational state and positional context of various components. It
 * provides a unified interface for monitoring the current condition and
 * status of different parts of the robot, ensuring a cohesive understanding
 * of the robot's overall configuration at any given time.
 */
public class Status {

    public static boolean competitionMode = false;

    public static Pose2D targetPose;
    public static Pose2D currentPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES,0);
    public static double currentHeading = 0;
    public static Constants.MOTIF motif;

    public static Constants.ALLIANCE alliance;

    /**
     * A map to store the status of each swerve servo.
     * The key is the integer, and the value is the ServoStatus enum.
     */
    public static ServoStatus[] swerveServoStatus = {ServoStatus.TARGET_REACHED, ServoStatus.TARGET_REACHED, ServoStatus.TARGET_REACHED, ServoStatus.TARGET_REACHED};

    public enum ServoStatus {
        TARGET_REACHED,
        MOVING
    }

    public static boolean robotHeadingTargetReached = false;

    public static boolean opModeIsActive = false;

    public static boolean lightsOn = false;
    public static boolean policeOn = false;

    public static boolean isDrivingActive = false;
}
