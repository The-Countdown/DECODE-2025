package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * The `Constants` class provides a centralized location for all the fixed values and
 * configurations used throughout the robot's code.
 */
@Config
public class Constants {
    public static final int NUM_SWERVE_MOTORS = 4;
    public static final int NUM_SWERVE_SERVOS = 4;
    public static final int NUM_SWERVE_ANALOGS = 4;

    /**
     * The angle offset for each swerve servo, used to correct any mechanical misalignments.
     * Index 4 is global, and the rest are in order (module 0, 1, 2, 3) @see {@link SwerveModule}.
     */
    public static final double[] SWERVE_SERVO_ANGLE_OFFSET = {0, 0, 0, 0, 0};

    /**
     * The desired servo angles for the swerve modules when in the stop formation.
     */
    public static final double[] SWERVE_STOP_FORMATION = {47.8901, -47.8901, 47.8901, -47.8901};
    /**
     * The desired servo angles for the swerve modules when in the rotation formation
     * These values are not exactly 45 degrees because the drivebase is not a perfect square
     */
    public static final double[] SWERVE_ROTATION_FORMATION = {-47.8901, 47.8901, -47.8901, 47.8901};
    /**
     * An array of power values for each swerve motor that sets them to no power.
     */
    public static final double[] SWERVE_NO_POWER = {0, 0, 0, 0};

    /**
     * PIDF values for the swerve drive servos.
     * These values will need to be tuned for the specific robot.
     */
    public static final double SWERVE_SERVO_KP = 0, SWERVE_SERVO_KI = 0, SWERVE_SERVO_KD = 0, SWERVE_SERVO_KF = 0;
    public static final double HEADING_KP = 0, HEADING_KI = 0, HEADING_KD = 0, HEADING_KF = 0;

    public static final double ANALOG_MAX_VOLTAGE = 3.3;

    public static final double HEADING_PIDF_TOLERANCE_DEGREES = 3;

    public static final double SWERVE_SERVO_PIDF_TOLERANCE_DEGREES = 1;

    public static final int SWERVE_MOTOR_RPM_CALCULATED_MAX = 5800;

    public static final int SWERVE_MOTOR_TPR = 28;

    public static final double SWERVE_MOTOR_TO_WHEEL_GEAR_RATIO = 6.74;

    public static final double WHEEL_DIAMETER_MM = 62;

    public static final int WHEEL_CALCULATED_MAX_RPM = 861;
}