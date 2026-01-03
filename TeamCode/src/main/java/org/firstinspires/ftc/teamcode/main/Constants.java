package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.HashMap;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * The `Constants` class provides a centralized location for all the fixed values and
 * configurations used throughout the robot's code.
 */
public class Constants {

    @Config
    public static class Game {
        public static double target = 0.5;
        public enum ALLIANCE {
            BLUE,
            RED
        }

        public static Pose2D ORIGIN = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        // Starting positions of the sets of 3 artifacts
        public static Pose2D POSPPG = new Pose2D(DistanceUnit.CM, -91.44, -109.22, AngleUnit.DEGREES, 0);
        public static Pose2D POSPGP = new Pose2D(DistanceUnit.CM, -30.48, -109.22, AngleUnit.DEGREES, 0);
        public static Pose2D POSGPP = new Pose2D(DistanceUnit.CM, 30.48, -109.22, AngleUnit.DEGREES, 0);

        public enum MOTIF {
            GPP,
            PGP,
            PPG,
            UNKNOWN
        }

        public enum ARTIFACT_COLOR{
            PURPLE,
            GREEN,
            NONE,
            UNKNOWN
        }
    }

    @Config
    public static class Robot {
        public static int MOTOR_UPDATE_TIME = 50;
        public static int SERVO_UPDATE_TIME = 100;
        public static int COLOR_UPDATE_TIME = 50;
        public static int ANALOG_UPDATE_TIME = 30;

        public static int SWERVE_MOTOR_UPDATE_TIME = 20;
        public static int SWERVE_SERVO_UPDATE_TIME = 10;
        public static int SWERVE_ANALOG_UPDATE_TIME = 5;

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        //public static double startingX = -156.21, startingY = -38.78, startingHeading = -180;
        public static double STARTING_X = -160.9725, STARTING_Y = -43.02125, STARTING_HEADING = -180;
        public static double GOALSIDE_STARTING_X = 142.5575, GOALSIDE_STARTING_Y = -18.415, GOALSIDE_STARTING_HEADING = 0;

        public static final int
                CONTROL_HUB_INDEX = 0,
                EXPANSION_HUB_INDEX = 1;

        public static final double
                WHEELBASE_WIDTH_MM = 432,
                WHEELBASE_LENGTH_MM = 367.067;

        public static final double
                WHEELBASE_ICR_X = WHEELBASE_LENGTH_MM / 2,
                WHEELBASE_ICR_Y = WHEELBASE_WIDTH_MM / 2;


        public static final double WHEEL_DIAMETER_MM = 62;

        public static final double WHEEL_CALCULATED_MAX_RPM = Swerve.MOTOR_RPM_TESTED_MAX * Swerve.MOTOR_TO_WHEEL_GEAR_RATIO;
        public static final double ROBOT_CALCULATED_MAX_SPEED_METERS_PER_SECOND = (2 * Math.PI * (WHEEL_DIAMETER_MM / 1000)) * (WHEEL_CALCULATED_MAX_RPM / 60);

        public static double CORNER_X = 0;
        public static double CORNER_Y = 0;
        public static double CORNER_ANGLE = 0;
    }

    @Config
    public static class Swerve {
        public static boolean SERVO_ANALOG_ACTIVE = true;

        public static final int
                NUM_MOTORS = 4,
                NUM_SERVOS = 4,
                NUM_ANALOGS = 4;

        public static final boolean[] MODULE_FLIPPED = new boolean[NUM_SERVOS];

        public static final double MODULE_FLIP_SWITCH_TOLERANCE_DEGREES = 2;

        public static final double MODULE_FLIP_SWITCH_ON = 90 + MODULE_FLIP_SWITCH_TOLERANCE_DEGREES;

        public static final double MODULE_FLIP_SWITCH_OFF = 90 - MODULE_FLIP_SWITCH_TOLERANCE_DEGREES;

        /**
         * The angle offset for each swerve servo, used to correct any mechanical misalignment.
         * Index 4 is global, and the rest are in order (module 0, 1, 2, 3) see {@link SwerveModule}.
         */
        public static double[] SERVO_ANGLE_OFFSET = {225, 17, 13, 58};

        /**
         * The desired servo angles for the swerve modules when in the rotation formation
         * These values are not exactly 45 degrees because the drivebase is not a perfect square
         */
        public static final double[] ROTATION_FORMATION_DEGREES = {
                HelperFunctions.normalizeAngle(Math.toDegrees(Math.atan2(Robot.WHEELBASE_ICR_Y, Robot.WHEELBASE_ICR_X))),
                HelperFunctions.normalizeAngle(Math.toDegrees(Math.atan2(-Robot.WHEELBASE_ICR_Y, Robot.WHEELBASE_ICR_X))),
                HelperFunctions.normalizeAngle(Math.toDegrees(Math.atan2(-Robot.WHEELBASE_ICR_Y, -Robot.WHEELBASE_ICR_X))),
                HelperFunctions.normalizeAngle(Math.toDegrees(Math.atan2(Robot.WHEELBASE_ICR_Y, -Robot.WHEELBASE_ICR_X))),
        };

        public static final double[] ROTATION_FORMATION_RADIANS = {
                Math.atan2(Robot.WHEELBASE_ICR_Y, Robot.WHEELBASE_ICR_X),
                Math.atan2(-Robot.WHEELBASE_ICR_Y, Robot.WHEELBASE_ICR_X),
                Math.atan2(-Robot.WHEELBASE_ICR_Y, -Robot.WHEELBASE_ICR_X),
                Math.atan2(Robot.WHEELBASE_ICR_Y, -Robot.WHEELBASE_ICR_X),
        };

        public static final double[] ROTATION_FORMATION_COSINES_RADIANS = {
                Math.cos(ROTATION_FORMATION_RADIANS[0]),
                Math.cos(ROTATION_FORMATION_RADIANS[1]),
                Math.cos(ROTATION_FORMATION_RADIANS[2]),
                Math.cos(ROTATION_FORMATION_RADIANS[3]),
        };

        public static final double[] ROTATION_FORMATION_SINES_RADIANS = {
                Math.sin(ROTATION_FORMATION_RADIANS[0]),
                Math.sin(ROTATION_FORMATION_RADIANS[1]),
                Math.sin(ROTATION_FORMATION_RADIANS[2]),
                Math.sin(ROTATION_FORMATION_RADIANS[3]),
        };

        /**
         * The desired servo angles for the swerve modules when in the stop formation.
         */
        public static final double[] STOP_FORMATION = {
                180 - ROTATION_FORMATION_DEGREES[0],
                180 - ROTATION_FORMATION_DEGREES[1],
                180 - ROTATION_FORMATION_DEGREES[2],
                180 - ROTATION_FORMATION_DEGREES[3],
        };

        /**
         * An array of power values for each swerve motor that sets them to no power.
         */
        public static final double[] NO_POWER = {0, 0, 0, 0};

        public static final double[] POWER_MULTIPLIER = {
                0, // Front Right
                0, // Front Left
                0, // Back Left
                0, // Back Right
        };

        /**
         * Scales the PIDF values for the swerve drive servos based off the speed of the motor being driven,
         * to compensate for the lesser friction when the robot is moving.
         */
        public static double SERVO_PIDF_SCALER = 0;

        /**
         * PIDF values for the swerve drive servos.
         * These values will need to be tuned.
         */
        public static double[]
                SERVO_KP = {0.005, 0.005, 0.006, 0.00348},
                SERVO_KD = {0, 0, 0, 0},
                SERVO_KF = {0.065, 0.065, 0.04, 0.06};

        public static double SERVO_PIDF_TOLERANCE_DEGREES = 1;

        public static final int MOTOR_RPM_TESTED_MAX = 5800;

        public static final int MOTOR_TICKS_PER_REVOLUTION = 28;

        public static final double MOTOR_TO_WHEEL_GEAR_RATIO = 0.0967;

        public static final int MOTOR_MAX_VELOCITY_TICKS_PER_SECOND = ((MOTOR_RPM_TESTED_MAX / 60) * MOTOR_TICKS_PER_REVOLUTION) - 328;
    }

    @Config
    public static class System {
        public static final int LOOP_AVERAGE_WINDOW_SIZE = 30;
        public static int TELEMETRY_UPDATE_INTERVAL_MS = 750;
        public static int TELEMETRY_COMP_UPDATE_INTERVAL_MS = 750;
        public static final double ANALOG_MAX_VOLTAGE = 3.3;
        public static final double GREEN_THRESHOLD = 70;
    }

    @Config
    public static class Control {
        public static double JOYSTICK_SCALER_EXPONENT = 1.3;

        //power per second
        public static double MAX_DRIVE_ACCELERATION = 5;
        public static double ZERO_POWER_TOLERANCE = 0.03;
    }

    @Config
    public static class Pathing {
        public static final double
                PINPOINT_X_OFFSET_MM = 177;
        public static final double PINPOINT_Y_OFFSET_MM = 7;

        public static final float PINPOINT_RESOLUTION = 19.68619876f;
        public static final GoBildaPinpointDriver.GoBildaOdometryPods
                PINPOINT_ODOM_POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public static final GoBildaPinpointDriver.EncoderDirection
                PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public static double
                HEADING_KP = 0.01,
                HEADING_KI = 0,
                HEADING_I_MAX = 0,
                HEADING_KD = 0.00003,
                HEADING_KF = 0.02;
        public static double
                LATITUDE_KP = 0.009,
                LATITUDE_KI = 0,
                LATITUDE_I_MAX = 0,
                LATITUDE_KD = 0,
                LATITUDE_KF = 0.03;
        public static double
                LONGITUDE_KP = 0.009,
                LONGITUDE_KI = 0,
                LONGITUDE_I_MAX = 0,
                LONGITUDE_KD = 0,
                LONGITUDE_KF = 0.03;
        public static double HEADING_PID_TOLERANCE_DEGREES = 10;
        public static double LATITUDE_PID_TOLERANCE_CM = 2;
        public static double LONGITUDE_PID_TOLERANCE_CM = 2;
        public static int PINPOINT_UPDATE_DELAY_MS = 50;
        public static double SWERVE_MAX_VELOCITY = 2200;
        public static double SWERVE_MAX_POWER = 1;
        public static int PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME = 5;
        public static int PATH_TIMEOUT_ERROR_MS = 500;
        public static NavigableMap<Double, Double> ACCELERATION_TABLE = new TreeMap<>();
    }

    @Config
    public static class LED {
        public static final HashMap<COLOR, LED_COLOR_VALUES> COLOR_MAP = new HashMap<COLOR, LED_COLOR_VALUES>() {{
            put(COLOR.OFF, new LED_COLOR_VALUES(500, 0.0));
            put(COLOR.RED, new LED_COLOR_VALUES(1100, 0.279));
            put(COLOR.ORANGE, new LED_COLOR_VALUES(1200, 0.333));
            put(COLOR.YELLOW, new LED_COLOR_VALUES(1300, 0.388));
            put(COLOR.SAGE, new LED_COLOR_VALUES(1400, 0.444));
            put(COLOR.GREEN, new LED_COLOR_VALUES(1500, 0.500));
            put(COLOR.AZURE, new LED_COLOR_VALUES(1600, 0.555));
            put(COLOR.BLUE, new LED_COLOR_VALUES(1700, 0.611));
            put(COLOR.INDIGO, new LED_COLOR_VALUES(1800, 0.666));
            put(COLOR.VIOLET, new LED_COLOR_VALUES(1900, 0.722));
            put(COLOR.WHITE, new LED_COLOR_VALUES(2500, 1.0));
        }};

        public enum COLOR {
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

        public static class LED_COLOR_VALUES {
            public final int MICROSECONDS; public final double ANALOG;
            public LED_COLOR_VALUES(int MICROSECONDS, double ANALOG) {
                this.MICROSECONDS = MICROSECONDS; this.ANALOG = ANALOG;
            }
        }
    }

    @Config
    public static class Turret {
        public static double TURRET_MAX = 0.78;
        public static double TURRET_NEUTRAL = 0.495;
        public static double TURRET_MIN = 0.215;
        public static double TURRET_SPEED_FACTOR = 0.0003;
        public static int FLYWHEEL_MAX_VELOCITY = 1720;
        public static int FLYWHEEL_MAX_POWER_ERROR = 200;
        public static double FLYWHEEL_CURVE = 4;
        public static double[] FLYWHEEL_SPEED_TABLE = {0.57, 0.595, 0.69, 0.715, 0.73, 0.82, 0.86};
        public static double[] FLYWHEEL_SPEED_TABLE_DISTANCES = {40, 56, 90, 95, 103, 127, 152};
        public static int FLYWHEEL_SPINUP_MS = 1500;
        public static double TRACK_GOAL_P = -0.2;
        public static double[] HOOD_PRESETS = {0.52, 0.07, 0.18};
        public static double TURRET_LIMIT_MIN_ANGLE = -90;
        public static double TURRET_LIMIT_MAX_ANGLE = 125;
        public static double FLYWHEEL_KP = 0.00022;
        public static double FLYWHEEL_KI = 0;
        public static double FLYWHEEL_KD = 0.0033;
        public static double FLYWHEEL_KF = 0;
    }

    @Config
    public static class Transfer {
        public static double DOWN = 0.295;
        public static double UP = 0.49;
    }

    @Config
    public static class Spindexer {
        public static int NUM_SLOTS = 3;
        public static double ANGLE_OFFSET = 45;
        public static double JAM_TIME_THRESHOLD = 1;
        public static double[] INTAKE_SLOT_ANGLES = {120, 240, 360};
        public static double[] BEFORE_TRANSFER_SLOT_ANGLES = {0, 0, 0};
        public static double[] AFTER_TRANSFER_SLOT_ANGLES = {0, 0, 0};
        public static int TIME_BETWEEN_BEAM_BREAK_AND_COLOR_SENSOR = 30;
        public static double TIME_BEAM_BREAK_TRUE_BEFORE_COLOR_SENSOR_CHECK = 0.1;
        public static double KP = 0.0043;
        public static double KI = 0;
        public static double KD = 0.001;
        public static double KF = 0;
        public static double DIST_TOLERANCE = 5;
        public static int FULL_EMPTY_SPINTIME = 2200;
    }

    @Config
    public static class Intake {
        public static double TOP_SPEED = 1;
        public static double BEST_INTAKE_SPEED = 1;
        public static double REVERSE_TOP_SPEED = 0.5;
    }
}
