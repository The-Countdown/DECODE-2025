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
            Unknown
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
        public static int COLOR_UPDATE_TIME = 150;
        public static int ANALOG_UPDATE_TIME = 30;

        public static int SWERVE_MOTOR_UPDATE_TIME = 20;
        public static int SWERVE_SERVO_UPDATE_TIME = 10;
        public static int SWERVE_ANALOG_UPDATE_TIME = 5;

        public static final IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        //public static double startingX = -142.5575, startingY = -18.415, startingHeading = -180;
        public static double startingX = -160.9725, startingY = -43.02125, startingHeading = -180; // Do NOT NOT ever change this heading unless you really know what you are doing.
        public static double GoalsideStartingX = 142.5575, GoalsideStartingY = -18.415, GoalsideStartingHeading = 0;

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

        public static final double ROBOT_CALCULATED_MAX_SPEED_METERS_PER_SECOND = (2*Math.PI * (WHEEL_DIAMETER_MM / 1000)) * (WHEEL_CALCULATED_MAX_RPM / 60);
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
        public static double turningRate = 0.20;
//        public static double
//                HEADING_KP = 0.0055,
//                HEADING_KI = 0,
//                HEADING_I_MAX = 0,
//                HEADING_KD = 0,
//                HEADING_KF = 0.03;
//        public static double
//                LATITUDE_KP = 0.007,
//                LATITUDE_KI = 0,
//                LATITUDE_I_MAX = 0,
//                LATITUDE_KD = 0,
//                LATITUDE_KF = 0.03;
//        public static double
//                LONGITUDE_KP = 0.007,
//                LONGITUDE_KI = 0,
//                LONGITUDE_I_MAX = 0,
//                LONGITUDE_KD = 0,
//                LONGITUDE_KF = 0.03;
        public static double
                HEADING_KP = 0.011,
                HEADING_KI = 0,
                HEADING_I_MAX = 0,
                HEADING_KD = 0,
                HEADING_KF = 0.03;
        public static double
                LATITUDE_KP = 0.011,
                LATITUDE_KI = 0,
                LATITUDE_I_MAX = 0,
                LATITUDE_KD = 0,
                LATITUDE_KF = 0.03;
        public static double
                LONGITUDE_KP = 0.011,
                LONGITUDE_KI = 0,
                LONGITUDE_I_MAX = 0,
                LONGITUDE_KD = 0,
                LONGITUDE_KF = 0.03;
        public static double HEADING_PID_TOLERANCE_DEGREES = 1;
        public static double LATITUDE_PID_TOLERANCE_CM = 1;
        public static double LONGITUDE_PID_TOLERANCE_CM = 1;
        public static int PINPOINT_UPDATE_DELAY_MS = 50;
        public static int LIMELIGHT_UPDATE_AVERAGING_MS = 500;
        public static double SWERVE_MAX_VELOCITY = 2000;
        public static double SWERVE_MAX_POWER = 1;
        public static double SWERVE_MAX_SPEED_CM_PER_SECOND = (Constants.Pathing.SWERVE_MAX_VELOCITY / Constants.Swerve.MOTOR_TICKS_PER_REVOLUTION) * Constants.Swerve.MOTOR_TO_WHEEL_GEAR_RATIO * 2 * Math.PI * (Constants.Robot.WHEEL_DIAMETER_MM/10);
        public static double SWERVE_MAX_LINEAR_SPEED_CM_PER_SECOND_WHILE_ROTATING_MULTIPLIER = 0.67;
        public static int PATH_TIMEOUT_ERROR_MS = 500;
        public static int PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME = 2;

        //distance in cm, time in seconds
        public static final NavigableMap<Double, Double> ACCELERATION_TABLE =
                new TreeMap<>();

        static {
            ACCELERATION_TABLE.put(0.0, 0.001119417);
            ACCELERATION_TABLE.put(1.22E-4, 0.010142418);
            ACCELERATION_TABLE.put(1.27E-4, 0.256907901);
            ACCELERATION_TABLE.put(0.02387704717, 0.289432529);
            ACCELERATION_TABLE.put(0.0844440415, 0.302068114);
            ACCELERATION_TABLE.put(0.2080164947, 0.316093782);
            ACCELERATION_TABLE.put(0.2785114319, 0.330841325);
            ACCELERATION_TABLE.put(0.4150996399, 0.341729826);
            ACCELERATION_TABLE.put(0.5273043356, 0.355914453);
            ACCELERATION_TABLE.put(0.7699388166, 0.370747746);
            ACCELERATION_TABLE.put(1.096985101, 0.383611122);
            ACCELERATION_TABLE.put(1.440582667, 0.404621916);
            ACCELERATION_TABLE.put(1.99949841, 0.41499825);
            ACCELERATION_TABLE.put(2.555890117, 0.426804043);
            ACCELERATION_TABLE.put(3.081549089, 0.441006169);
            ACCELERATION_TABLE.put(3.732233072, 0.452901212);
            ACCELERATION_TABLE.put(4.328778494, 0.463218338);
            ACCELERATION_TABLE.put(5.076926861, 0.480776965);
            ACCELERATION_TABLE.put(6.187361818, 0.496701967);
            ACCELERATION_TABLE.put(7.285340207, 0.512125593);
            ACCELERATION_TABLE.put(8.141033077, 0.523320344);
            ACCELERATION_TABLE.put(8.993919552, 0.535435887);
            ACCELERATION_TABLE.put(10.26841768, 0.549358014);
            ACCELERATION_TABLE.put(11.17095154, 0.568576516);
            ACCELERATION_TABLE.put(12.48396512, 0.573125058);
            ACCELERATION_TABLE.put(13.46030259, 0.592990185);
            ACCELERATION_TABLE.put(15.23856768, 0.607546103);
            ACCELERATION_TABLE.put(16.63236694, 0.621510229);
            ACCELERATION_TABLE.put(20.20446786, 0.654217149);
            ACCELERATION_TABLE.put(22.1621444, 0.674590943);
            ACCELERATION_TABLE.put(23.82482743, 0.688486236);
            ACCELERATION_TABLE.put(25.27798363, 0.701437404);
            ACCELERATION_TABLE.put(26.70006374, 0.718370114);
            ACCELERATION_TABLE.put(28.21921039, 0.730558865);
            ACCELERATION_TABLE.put(30.35498225, 0.745548492);
            ACCELERATION_TABLE.put(31.86522721, 0.755847826);
            ACCELERATION_TABLE.put(33.40366797, 0.782164912);
            ACCELERATION_TABLE.put(35.39930415, 0.791506996);
            ACCELERATION_TABLE.put(37.61517333, 0.807065373);
            ACCELERATION_TABLE.put(39.34212084, 0.829578542);
            ACCELERATION_TABLE.put(41.63619712, 0.840949751);
            ACCELERATION_TABLE.put(43.71109329, 0.849803294);
            ACCELERATION_TABLE.put(44.94755544, 0.867302712);
            ACCELERATION_TABLE.put(46.97862387, 0.877710838);
            ACCELERATION_TABLE.put(48.38007119, 0.891334298);
            ACCELERATION_TABLE.put(50.15843643, 0.906909883);
            ACCELERATION_TABLE.put(52.16376289, 0.922277218);
            ACCELERATION_TABLE.put(54.17024877, 0.934941094);
            ACCELERATION_TABLE.put(55.89481438, 0.950341971);
            ACCELERATION_TABLE.put(57.85517037, 0.963317347);
            ACCELERATION_TABLE.put(59.42051888, 0.97461739);
            ACCELERATION_TABLE.put(61.42285705, 0.994403183);
            ACCELERATION_TABLE.put(63.54781536, 1.013996769);
            ACCELERATION_TABLE.put(65.65676567, 1.021599936);
            ACCELERATION_TABLE.put(67.8459511, 1.04058248);
            ACCELERATION_TABLE.put(69.83109745, 1.053862356);
            ACCELERATION_TABLE.put(72.14480638, 1.068342149);
            ACCELERATION_TABLE.put(73.58416128, 1.081564275);
            ACCELERATION_TABLE.put(75.39251063, 1.090707151);
            ACCELERATION_TABLE.put(76.69092498, 1.111778028);
            ACCELERATION_TABLE.put(78.88724105, 1.116347862);
            ACCELERATION_TABLE.put(80.49663221, 1.133063864);
            ACCELERATION_TABLE.put(82.81404458, 1.147285824);
            ACCELERATION_TABLE.put(85.04234662, 1.167323617);
            ACCELERATION_TABLE.put(87.15059188, 1.179070785);
            ACCELERATION_TABLE.put(89.15502968, 1.193455203);
            ACCELERATION_TABLE.put(91.17662683, 1.209172247);
            ACCELERATION_TABLE.put(93.15733497, 1.223684706);
            ACCELERATION_TABLE.put(95.24081723, 1.237119749);
            ACCELERATION_TABLE.put(96.94475969, 1.252303043);
            ACCELERATION_TABLE.put(98.55988757, 1.261650377);
            ACCELERATION_TABLE.put(100.5334595, 1.274536212);
            ACCELERATION_TABLE.put(102.1004999, 1.284728796);
            ACCELERATION_TABLE.put(103.948067, 1.301930131);
            ACCELERATION_TABLE.put(105.8149488, 1.316301716);
            ACCELERATION_TABLE.put(107.7659278, 1.325648175);
            ACCELERATION_TABLE.put(109.5482043, 1.34160176);
            ACCELERATION_TABLE.put(111.2886082, 1.354291595);
            ACCELERATION_TABLE.put(113.062733, 1.369604096);
            ACCELERATION_TABLE.put(114.6318929, 1.379925597);
            ACCELERATION_TABLE.put(116.6176394, 1.394413849);
            ACCELERATION_TABLE.put(118.5789105, 1.407025808);
            ACCELERATION_TABLE.put(120.7288266, 1.42903556);
            ACCELERATION_TABLE.put(122.5963226, 1.437101895);
            ACCELERATION_TABLE.put(124.6320039, 1.450800021);
            ACCELERATION_TABLE.put(126.4185811, 1.469840898);
            ACCELERATION_TABLE.put(128.7374866, 1.485793899);
            ACCELERATION_TABLE.put(130.8923659, 1.497098026);
            ACCELERATION_TABLE.put(132.6222743, 1.513445652);
            ACCELERATION_TABLE.put(134.8310892, 1.525264862);
            ACCELERATION_TABLE.put(136.9713889, 1.541865072);
            ACCELERATION_TABLE.put(138.7013669, 1.55403924);
            ACCELERATION_TABLE.put(140.4547351, 1.563229657);
            ACCELERATION_TABLE.put(142.1246875, 1.577186492);
            ACCELERATION_TABLE.put(143.7875081, 1.588724243);
            ACCELERATION_TABLE.put(145.4450362, 1.603298245);
            ACCELERATION_TABLE.put(147.5674636, 1.614530912);
            ACCELERATION_TABLE.put(149.4539823, 1.629337955);
            ACCELERATION_TABLE.put(151.3444447, 1.647650541);
            ACCELERATION_TABLE.put(153.8515268, 1.659675084);
            ACCELERATION_TABLE.put(156.0453913, 1.679376586);
            ACCELERATION_TABLE.put(157.7272967, 1.688017795);
            ACCELERATION_TABLE.put(159.8656019, 1.707087547);
            ACCELERATION_TABLE.put(161.8972524, 1.719041506);
            ACCELERATION_TABLE.put(163.9156222, 1.734041924);
            ACCELERATION_TABLE.put(165.2388064, 1.751384718);
            ACCELERATION_TABLE.put(167.3004275, 1.765063011);
            ACCELERATION_TABLE.put(169.0743778, 1.780410804);
            ACCELERATION_TABLE.put(170.8597274, 1.79030968);
            ACCELERATION_TABLE.put(172.0673581, 1.813683849);
            ACCELERATION_TABLE.put(173.9384587, 1.823523225);
            ACCELERATION_TABLE.put(176.0765487, 1.842899519);
            ACCELERATION_TABLE.put(177.7886224, 1.857168145);
            ACCELERATION_TABLE.put(179.3433807, 1.870718396);
        }
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
        public static double TURRET_SPEED_FACTOR = 0.001;
        public static double FLYWHEEL_TOP_SPEED = 0.95;
        public static int FLYWHEEL_MAX_VELOCITY = 1720;
        public static int FLYWHEEL_MAX_POWER_ERROR = 200;
        public static double FLYWHEEL_CURVE = 4;
        public static double[] FLYWHEEL_SPEED_TABLE = {0.57, 0.595, 0.7, 0.715, 0.73, 0.813, 0.84};
        public static double[] FLYWHEEL_SPEED_TABLE_DISTANCES = {40, 56, 90, 95, 103, 127, 152};
        public static int FLYWHEEL_SPINUP_MS = 1000;
        public static double TRACK_GOAL_P = -0.2;
        public static double[] HOOD_PRESETS = {0.45, 0, 0};
        public static double TURRET_LIMIT_MIN_ANGLE = -90;
        public static double TURRET_LIMIT_MAX_ANGLE = 125;
        public static double TURRET_LIMIT_MIN_SERVO = TURRET_LIMIT_MIN_ANGLE / 355; //-0.253521127
        public static double TURRET_LIMIT_MAX_SERVO = TURRET_LIMIT_MAX_ANGLE / 355; //0.352112676
        public static double TURRET_LIMIT_MIN = (TURRET_LIMIT_MIN_SERVO * 2) - 1; //-1.507042254
        public static double TURRET_LIMIT_MAX = (TURRET_LIMIT_MAX_SERVO * 2) - 1; //-0.295774648
        public static double FLYWHEEL_POWER_AUTO_FAR = 0.605;
        public static double FLYWHEEL_POWER_AUTO_MIDDLE = 0.49;
        public static double FLYWHEEL_POWER_AUTO_NEAR = 0.48;
        public static double FLYWHEEL_P = 0.00019;
        public static double FLYWHEEL_I = 0.0;
        public static double FLYWHEEL_D = 0.0033;
        public static double FLYWHEEL_F = 0.0;
    }

    @Config
    public static class Transfer {
        public static int FLIP_TIME = 300;
        public static double DOWN = 0.295;
        public static double UP = 0.49;
    }

    @Config
    public static class Spindexer {
        public static int NUM_SLOTS = 3;
        public static double ANGLE_OFFSET = 45;
        public static double JAM_TIME_THRESHOLD = 1;
        public static double axonTestAngle = 0;
        public static double[] TRANSFER_SLOT_ANGLES = {60, 180, 300};
        public static double[] INTAKE_SLOT_ANGLES = {120, 240, 360};
        public static int COLOR_SENSE_TIME = 30;
        public static double KP = 0.0018;
        public static double KI = 0;
        public static double KD = 0.00015;
        public static double KF = 0.015;
        public static double DIST_TOLERANCE = 6;
        public static double BEAM_TIMER_TOLERANCE = 0.15;
        public static int FULL_EMPTY_SPINTIME = 2000;
    }

    @Config
    public static class Intake {
        public static double TOP_SPEED = 0.67;
        public static double BEST_INTAKE_SPEED = 0.67;
        public static double SPIN_ERROR_SPEED = 0.4;
        public static double REVERSE_TOP_SPEED = 0.5;
        public static int RUNTIME_MS = 800;
        public static double DELAY_SECONDS = 0.2;
    }
}
