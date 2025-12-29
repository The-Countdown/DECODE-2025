package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

/**
 * The `Status` class serves as a centralized repository for tracking the
 * operational state and positional context of various components. It
 * provides a unified interface for monitoring the current condition and
 * status of different parts of the robot, ensuring a cohesive understanding
 * of the robot's overall configuration at any given time.
 */
public class Status {
    public static Pose2D targetPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES,0);
    public static Pose2D currentPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES,0);
    public static double currentHeading = 0;
    public static Constants.Game.MOTIF  motif = Constants.Game.MOTIF.UNKNOWN;
    public static Constants.Game.ALLIANCE alliance = Constants.Game.ALLIANCE.RED;
    public static boolean competitionMode;

    /**
     * A map to store the status of each swerve servo.
     * The key is the integer, and the value is the ServoStatus enum.
     */

    public static Pose2D cornerResetPose;
    public static Pose2D goalPose;
    public static Pose2D startingPose;
    public static Pose2D goalsideStartingPose;
    public static boolean robotHeadingTargetReached = false;
    public static boolean robotLatitudeTargetReached = false;
    public static boolean robotLongitudeTargetReached = false;

    public static boolean opModeIsActive = false;
    public static boolean wentBackToStart = false;

    public static boolean lightsOn = false;
    public static Constants.LED.COLOR currentLightColor;

    public static boolean isDrivingActive = false;
    public static boolean fieldOriented = false;

    public static boolean intakeToggle = false;
    public static final GamepadWrapper.ButtonReader turretToggleButton = new GamepadWrapper.ButtonReader();
    public static boolean turretToggle = false;
    public static boolean flywheelAtTargetSpeed = true;
    public static boolean flywheelToggle = false;
    public static boolean manualControl = false;
}
