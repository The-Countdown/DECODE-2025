package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import java.util.HashMap;

/**
import java.util.Map;
 * The `Constants` class provides a centralized location for all the fixed values and
 * configurations used throughout the robot's code.
 */
@Config
public class Constants {
    public static final int
            NUM_SWERVE_MOTORS = 4,
            NUM_SWERVE_SERVOS = 4,
            NUM_SWERVE_ANALOGS = 4;

    /**
     * The angle offset for each swerve servo, used to correct any mechanical misalignments.
     * Index 4 is global, and the rest are in order (module 0, 1, 2, 3) @see {@link SwerveModule}.
     */
    // TODO: Tune if needed
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
     * These values will need to be tuned.
     */
    // TODO: Tune
    public static final double
            SWERVE_SERVO_KP = 0,
            SWERVE_SERVO_KI = 0,
            SWERVE_SERVO_KD = 0,
            SWERVE_SERVO_KF = 0;
    // TODO: Tune
    public static final double
            HEADING_KP = 0,
            HEADING_KI = 0,
            HEADING_KD = 0;

    // TODO: Find these values
    public static final double
            PINPOINT_X_OFFSET_MM = 0,
            PINPOINT_Y_OFFSET_MM = 0;
    public static final GoBildaPinpoint.GoBildaOdometryPods
            PINPOINT_ODOM_POD = GoBildaPinpoint.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static final GoBildaPinpoint.EncoderDirection
            PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpoint.EncoderDirection.FORWARD,
            PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpoint.EncoderDirection.FORWARD;

    public static final double ANALOG_MAX_VOLTAGE = 3.3;

    // TODO: Tune if needed
    public static final double HEADING_PIDF_TOLERANCE_DEGREES = 3;

    // TODO: Tune if needed
    public static final double SWERVE_SERVO_PIDF_TOLERANCE_DEGREES = 1;

    public static final int SWERVE_MOTOR_RPM_CALCULATED_MAX = 5800;

    public static final int SWERVE_MOTOR_TPR = 28;

    public static final double SWERVE_MOTOR_TO_WHEEL_GEAR_RATIO = 6.74;

    public static final double WHEEL_DIAMETER_MM = 62;

    public static final int WHEEL_CALCULATED_MAX_RPM = 861;

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
        WHITE;
    }

    public static final HashMap<LED_COLOR, LED_COLOR_VALUES> LED_COLOR_MAP = new HashMap<LED_COLOR, LED_COLOR_VALUES>() {{
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
}