package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;

import java.util.HashMap;

/**
 * The `Constants` class provides a centralized location for all the fixed values and
 * configurations used throughout the robot's code.
 */
@Config
public class Constants {
    public static final IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    public static boolean MECANUM_ACTIVE = false;
    public static boolean TURRET_ACTIVE = false;
    public static boolean SERVO_ANALOG_ACTIVE = true;

    public static final int
            CONTROL_HUB_INDEX = 0,
            EXPANSION_HUB_INDEX = 1;

    public static final int LOOP_AVERAGE_WINDOW_SIZE = 30;

    public static final int
            NUM_SWERVE_MOTORS = 4,
            NUM_SWERVE_SERVOS = 4,
            NUM_SWERVE_ANALOGS = 4;

    public static final double
            WHEELBASE_WIDTH_MM = 320,
            WHEELBASE_LENGTH_MM = 285.68668;

    public static final double
            WHEELBASE_ICR_X = WHEELBASE_LENGTH_MM / 2,
            WHEELBASE_ICR_Y = WHEELBASE_WIDTH_MM / 2;

    /**
     * The angle offset for each swerve servo, used to correct any mechanical misalignment.
     * Index 4 is global, and the rest are in order (module 0, 1, 2, 3) see {@link SwerveModule}.
     */
    // TODO: Tune if needed
    public static double[] SWERVE_SERVO_ANGLE_OFFSET = {0, 0, 0, 0, 0};

    /**
     * The desired servo angles for the swerve modules when in the rotation formation
     * These values are not exactly 45 degrees because the drivebase is not a perfect square
     */
    public static final double[] SWERVE_ROTATION_FORMATION_DEGREES = {
            normalizeAngle(Math.toDegrees(Math.atan2(WHEELBASE_ICR_Y, WHEELBASE_ICR_X))),
            normalizeAngle(Math.toDegrees(Math.atan2(-WHEELBASE_ICR_Y, WHEELBASE_ICR_X))),
            normalizeAngle(Math.toDegrees(Math.atan2(-WHEELBASE_ICR_Y, -WHEELBASE_ICR_X))),
            normalizeAngle(Math.toDegrees(Math.atan2(WHEELBASE_ICR_Y, -WHEELBASE_ICR_X)))
    };

    public static final double[] SWERVE_ROTATION_FORMATION_RADIANS = {
            Math.atan2(WHEELBASE_ICR_Y, WHEELBASE_ICR_X),
            Math.atan2(-WHEELBASE_ICR_Y, WHEELBASE_ICR_X),
            Math.atan2(-WHEELBASE_ICR_Y, -WHEELBASE_ICR_X),
            Math.atan2(WHEELBASE_ICR_Y, -WHEELBASE_ICR_X)
    };

    public static final double[] SWERVE_ROTATION_FORMATION_COSINES_RADIANS = {
            Math.cos(SWERVE_ROTATION_FORMATION_RADIANS[0]),
            Math.cos(SWERVE_ROTATION_FORMATION_RADIANS[1]),
            Math.cos(SWERVE_ROTATION_FORMATION_RADIANS[2]),
            Math.cos(SWERVE_ROTATION_FORMATION_RADIANS[3])
    };

    public static final double[] SWERVE_ROTATION_FORMATION_SINES_RADIANS = {
            Math.sin(SWERVE_ROTATION_FORMATION_RADIANS[0]),
            Math.sin(SWERVE_ROTATION_FORMATION_RADIANS[1]),
            Math.sin(SWERVE_ROTATION_FORMATION_RADIANS[2]),
            Math.sin(SWERVE_ROTATION_FORMATION_RADIANS[3])
    };

    /**
     * The desired servo angles for the swerve modules when in the stop formation.
     */
    public static final double[] SWERVE_STOP_FORMATION = {
            180 - SWERVE_ROTATION_FORMATION_DEGREES[0],
            180 - SWERVE_ROTATION_FORMATION_DEGREES[1],
            180 - SWERVE_ROTATION_FORMATION_DEGREES[2],
            180 - SWERVE_ROTATION_FORMATION_DEGREES[3]
    };

    /**
     * An array of power values for each swerve motor that sets them to no power.
     */
    public static final double[] SWERVE_NO_POWER = {0, 0, 0, 0};

    /**
     * PIDF values for the swerve drive servos.
     * These values will need to be tuned.
     */
    // TODO: Tune
    public static double
            SWERVE_SERVO_KP = 0.01,
            SWERVE_SERVO_KI = 0,
                SWERVE_SERVO_I_MAX = 0,
            SWERVE_SERVO_KD = 0.0003,
            SWERVE_SERVO_KF = 0;

    // TODO: Tune
    public static double
            HEADING_KP = 0,
            HEADING_KI = 0,
                HEADING_I_MAX = 0,
            HEADING_KD = 0;

    public static final double
            PINPOINT_X_OFFSET_MM = 145,
            PINPOINT_Y_OFFSET_MM = -145;

    public static final GoBildaPinpoint.GoBildaOdometryPods
            PINPOINT_ODOM_POD = GoBildaPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final GoBildaPinpoint.EncoderDirection
            PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpoint.EncoderDirection.FORWARD,
            PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpoint.EncoderDirection.REVERSED;

    public static final double ANALOG_MAX_VOLTAGE = 3.3;

    // TODO: Tune if needed
    public static double HEADING_PID_TOLERANCE_DEGREES = 2;

    // TODO: Tune if needed
    public static double SWERVE_SERVO_PIDF_TOLERANCE_DEGREES = 1;

    public static final int SWERVE_MOTOR_RPM_CALCULATED_MAX = 5800;

    public static final int SWERVE_MOTOR_TPR = 28;

    public static final double SWERVE_MOTOR_TO_WHEEL_GEAR_RATIO = 6.74;

    public static final double WHEEL_DIAMETER_MM = 62;

    public static final int WHEEL_CALCULATED_MAX_RPM = 861;

    // TODO: Tune if needed
    public static double JOYSTICK_SCALER_EXPONENT = 1.33;

    //power per second
    public static double MAX_DRIVE_ACCELERATION = 1;

    public static double ZERO_POWER_TOLERANCE = 0.03;

    public static double TURRET_ACCELERATION_TIME_SECONDS = 3;

    // will not update if TURRET_ACCELERATION_TIME_SECONDS is changed in ftc dashboard
    public static double TURRET_ACCELERATION_MULTIPLIER_NANO = TURRET_ACCELERATION_TIME_SECONDS * 1e9 * 2781;

    public enum LED_COLOR {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public static final HashMap<LED_COLOR, LED_COLOR_VALUES> LED_COLOR_MAP = new HashMap<LED_COLOR, LED_COLOR_VALUES>() {{
        put(LED_COLOR.OFF, new LED_COLOR_VALUES(500, 0.0));
        put(LED_COLOR.RED, new LED_COLOR_VALUES(1100, 0.279));
        put(LED_COLOR.ORANGE, new LED_COLOR_VALUES(1200, 0.333));
        put(LED_COLOR.YELLOW, new LED_COLOR_VALUES(1300, 0.388));
        put(LED_COLOR.SAGE, new LED_COLOR_VALUES(1400, 0.444));
        put(LED_COLOR.GREEN, new LED_COLOR_VALUES(1500, 0.500));
        put(LED_COLOR.AZURE, new LED_COLOR_VALUES(1600, 0.555));
        put(LED_COLOR.BLUE, new LED_COLOR_VALUES(1700, 0.611));
        put(LED_COLOR.INDIGO, new LED_COLOR_VALUES(1800, 0.666));
        put(LED_COLOR.VIOLET, new LED_COLOR_VALUES(1900, 0.722));
        put(LED_COLOR.WHITE, new LED_COLOR_VALUES(2500, 1.0));
    }};
    public static class LED_COLOR_VALUES {
        public final int MICROSECONDS; public final double ANALOG;
        public LED_COLOR_VALUES(int MICROSECONDS, double ANALOG) {
            this.MICROSECONDS = MICROSECONDS; this.ANALOG = ANALOG;
        }
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
    private static double normalizeAngle(double angle) {
        // Check if the angle is already in the desired range.
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [0, 360).
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