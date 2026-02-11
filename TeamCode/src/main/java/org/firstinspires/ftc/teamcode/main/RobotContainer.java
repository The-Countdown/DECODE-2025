package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.drivetrain.SwervePDF;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.FeedForward;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.HeadingPID;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LatitudePID;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LongitudePID;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathingUpdater;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLighting;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLogic;
import org.firstinspires.ftc.teamcode.subsystems.PositionProvider;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;
import org.firstinspires.ftc.teamcode.util.TelemetryLogger;
import org.firstinspires.ftc.teamcode.hardware.BetterIMU;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * The Robot class serves as the central manager for all robot hardware and high-level operations.
 * It initializes and provides access to key components such as the IMU, Limelight, swerve drive modules,
 * and associated control mechanisms. This class also manages the interaction with the FTC SDK, handles
 * asynchronous tasks through a Handler, and provides utility methods for common robot operations.
 * The {@link HardwareDevices} nested class contains static members that describe the different types of hardware the robot uses.
 * This class acts as the main interface for controlling the robot, offering a structured and organized
 * approach to managing complex robotic systems.
 */
public class RobotContainer {
    public double startTimeMs = System.currentTimeMillis();
    public OpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public boolean isRunning = false;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
    public final GamepadWrapper.ButtonReader beamBreakToggleButton = new GamepadWrapper.ButtonReader();
    public final Map<String, LinkedList<Double>> loopTimesMap = new HashMap<>();
    public final Map<String, ElapsedTime> loopTimers = new HashMap<>();
    private final ElapsedTime telemetryLoopTimer = new ElapsedTime();
    private final ArrayList<String> eventTelemetry = new ArrayList<>();
    private final ArrayList<String> eventTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> eventTelemetryValues = new ArrayList<>();
    private Map<String, Object> currentLoopData = new HashMap<>();
    public final SwerveModule[] swerveModules = new SwerveModule[Constants.Swerve.NUM_MOTORS];
    public SwervePDF[] swerveServosPDF = new SwervePDF[Constants.Swerve.NUM_SERVOS];
    public TelemetryLogger telemetryLogger;
    public LocalizationUpdater localizationUpdater;
    public DrivetrainUpdater drivetrainUpdater;
    public PathingUpdater pathingUpdater;
    public PathPlanner pathPlanner;
    public FeedForward feedForward;
    public LimelightLogic limelightLogic;
    public PositionProvider positionProvider;
    public DelayedActionManager delayedActionManager = new DelayedActionManager(this);
    public Drivetrain drivetrain;
    public HeadingPID headingPID;
    public LatitudePID latitudePID;
    public LongitudePID longitudePID;
    public IndicatorLighting.Light indicatorLightFront;
    public IndicatorLighting.Light indicatorLightBack;
    public IndicatorLighting.Group allIndicatorLights;
    public Turret turret;
    public Intake intake;
    public Spindexer spindexer;
    public double controlHubVoltage;
    public double expansionHubVoltage;
    public double controlHubCurrent;
    public double expansionHubCurrent;

//    public ArrayList<ArrayList<String>> telemetryCache;
    public ArrayList<String> telemetryHeaderList;
    public Map<String, ArrayList<String>> telemetryCache;

    public double CURRENT_LOOP_TIME_MS;
    public double PREV_LOOP_TIME_MS;

    public static BNO055IMU.Parameters betterIMUP = new BNO055IMU.Parameters();

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static LynxModule controlHub;
        public static LynxModule expansionHub;
        public static IMU imu;
        public static BetterIMU betterIMU;

        public static GoBildaPinpointDriver pinpoint;
        public static Limelight3A limelight;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLightFront;
        public static ServoImplEx indicatorLightBack;

        // Gobilda 5000 Series
        public static BetterDcMotor[] swerveMotors = new BetterDcMotor[Constants.Swerve.NUM_MOTORS];
            public static String[] motorNames = new String[Constants.Swerve.NUM_MOTORS];

        // Axon Mini+
        public static BetterServo[] swerveServos = new BetterServo[Constants.Swerve.NUM_SERVOS];
            public static String[] servoNames = new String[Constants.Swerve.NUM_SERVOS];

        public static BetterAnalogInput[] swerveAnalogs = new BetterAnalogInput[Constants.Swerve.NUM_ANALOGS];
            public static String[] analogNames = new String[Constants.Swerve.NUM_ANALOGS];

        // Turret
        public static BetterDcMotor flyWheelMotorMaster;
        public static BetterDcMotor flyWheelMotorSlave;
        public static BetterServo turretServoMaster;
        public static BetterServo turretServoSlave;
        public static BetterServo hoodServo;

        // Spindexer
        public static BetterCRServo spindexerServoMaster;
        public static BetterCRServo spindexerServoSlave;
        public static LinkedServos spindexerServos;
        public static BetterAnalogInput spindexerAnalog;

        // Intake
        public static BetterDcMotor intakeMotor;

        //sensors
        public static BetterColorSensor colorSensor;
        public static RevTouchSensor beamBreak;
    }

    public RobotContainer(OpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
//        this.telemetry = opMode.telemetry;
        this.telemetryCache = new HashMap<>();
        this.telemetryHeaderList = new ArrayList<>();

        HardwareDevices.imu = getHardwareDevice(IMU.class, "imu");
        HardwareDevices.imu.initialize(Constants.Robot.IMU_PARAMETERS);

        HardwareDevices.betterIMU = new BetterIMU(getHardwareDevice(AdafruitBNO055IMU.class, "betterIMU"));
        betterIMUP.mode = BNO055IMU.SensorMode.IMU;
        betterIMUP.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        betterIMUP.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        betterIMUP.calibrationDataFile = "AdafruitIMUCalibration.json";
        // parameters.loggingEnabled      = true;
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        HardwareDevices.betterIMU.initialize(betterIMUP);
        Thread.sleep(800); // Ensure that the IMU has some still time so that it will auto calibrate at zero.

        HardwareDevices.pinpoint = getHardwareDevice(GoBildaPinpointDriver.class, "pinpoint");
        HardwareDevices.pinpoint.setOffsets(Constants.Pathing.PINPOINT_X_OFFSET_MM, Constants.Pathing.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        HardwareDevices.pinpoint.setEncoderResolution(Constants.Pathing.PINPOINT_ODOM_POD);
        HardwareDevices.pinpoint.setEncoderDirections(Constants.Pathing.PINPOINT_X_ENCODER_DIRECTION, Constants.Pathing.PINPOINT_Y_ENCODER_DIRECTION);

        HardwareDevices.limelight = getHardwareDevice(Limelight3A.class, "limelight");

        HardwareDevices.indicatorLightFront = getHardwareDevice(ServoImplEx.class, "indicatorLightFront");
        HardwareDevices.indicatorLightBack = getHardwareDevice(ServoImplEx.class, "indicatorLightBack");

        for (int i = 0; i < swerveModules.length; i++) {
            HardwareDevices.motorNames[i] = "swerveMotor" + (i);
            HardwareDevices.swerveMotors[i] = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, HardwareDevices.motorNames[i]), Constants.Robot.SWERVE_MOTOR_UPDATE_TIME);
            HardwareDevices.servoNames[i] = "swerveServo" + (i);
            HardwareDevices.swerveServos[i] = new BetterServo(hardwareMap.get(ServoImplEx.class, HardwareDevices.servoNames[i]), Constants.Robot.SWERVE_SERVO_UPDATE_TIME);
            HardwareDevices.analogNames[i] = "swerveAnalog" + (i);
            HardwareDevices.swerveAnalogs[i] = new BetterAnalogInput(hardwareMap.get(AnalogInput.class, HardwareDevices.analogNames[i]), Constants.Robot.SWERVE_ANALOG_UPDATE_TIME);

            if (i == 0 || i == 2) {
                HardwareDevices.swerveMotors[i].setDirection(DcMotorImplEx.Direction.REVERSE);
            }
            HardwareDevices.swerveMotors[i].setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
            HardwareDevices.swerveMotors[i].setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

            swerveServosPDF[i] = new SwervePDF(this, i);
            swerveModules[i] = new SwerveModule(this, HardwareDevices.swerveMotors[i], HardwareDevices.swerveServos[i], swerveServosPDF[i], HardwareDevices.swerveAnalogs[i], Constants.Swerve.POWER_MULTIPLIER[i], i);  //is it best to pass in a constant?
        }

        HardwareDevices.turretServoMaster = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoMaster"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.turretServoSlave = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoSlave"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.hoodServo = new BetterServo(getHardwareDevice(ServoImplEx.class, "hoodServo"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.spindexerServoMaster = new BetterCRServo(getHardwareDevice(CRServoImplEx.class, "spindexerServoMaster"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.spindexerServoSlave = new BetterCRServo(getHardwareDevice(CRServoImplEx.class, "spindexerServoSlave"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.spindexerAnalog = new BetterAnalogInput(getHardwareDevice(AnalogInput.class, "spindexerAnalog"), Constants.Robot.ANALOG_UPDATE_TIME);
        HardwareDevices.intakeMotor = new BetterDcMotor(getHardwareDevice(DcMotorImplEx.class, "intakeMotor"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.flyWheelMotorMaster = new BetterDcMotor(getHardwareDevice(DcMotorImplEx.class, "flyWheelMotorMaster"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.flyWheelMotorSlave = new BetterDcMotor(getHardwareDevice(DcMotorImplEx.class, "flyWheelMotorSlave"), Constants.Robot.MOTOR_UPDATE_TIME);

        //sensor
        HardwareDevices.colorSensor = new BetterColorSensor(getHardwareDevice(RevColorSensorV3.class, "colorSensor"), Constants.Robot.COLOR_UPDATE_TIME);
        HardwareDevices.beamBreak = getHardwareDevice(RevTouchSensor.class, "beamBreak");


        LinkedMotors flyWheelMotors = new LinkedMotors(HardwareDevices.flyWheelMotorMaster, HardwareDevices.flyWheelMotorSlave);
        HardwareDevices.flyWheelMotorSlave.setDirection(DcMotorImplEx.Direction.REVERSE);
        flyWheelMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LinkedServos turretServos = new LinkedServos(HardwareDevices.turretServoMaster, HardwareDevices.turretServoSlave);

        pathPlanner = new PathPlanner(telemetry, this);
        feedForward = new FeedForward();
        limelightLogic = new LimelightLogic(this, telemetry, HardwareDevices.limelight);
        positionProvider = new PositionProvider(this, limelightLogic, HardwareDevices.pinpoint);
        turret = new Turret(this, flyWheelMotors, HardwareDevices.hoodServo, turretServos);
        HardwareDevices.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(this, HardwareDevices.intakeMotor);
        HardwareDevices.spindexerServos = new LinkedServos(HardwareDevices.spindexerServoMaster, HardwareDevices.spindexerServoSlave);
        spindexer = new Spindexer(this, HardwareDevices.spindexerServos, HardwareDevices.spindexerAnalog, HardwareDevices.colorSensor);

        indicatorLightFront = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightFront);
        indicatorLightBack = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightBack);
        allIndicatorLights = new IndicatorLighting.Group(this);
        allIndicatorLights.addLight(indicatorLightFront);
        allIndicatorLights.addLight(indicatorLightBack);

        drivetrain = new Drivetrain(this);
        headingPID = new HeadingPID();
        latitudePID = new LatitudePID();
        longitudePID = new LongitudePID();

        registerLoopTimer("teleOp");
        registerLoopTimer("drivetrainUpdater");
        registerLoopTimer("pinpointUpdater");
        registerLoopTimer("telemetryLogger");
    }

    public void init() {
        this.isRunning = true;
        RobotContainer.HardwareDevices.imu.resetYaw();
        this.pathPlanner = new PathPlanner(this.telemetry, this);
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        HardwareDevices.controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        HardwareDevices.expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            // hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            // hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        // Reset the IMU angle
        HardwareDevices.betterIMU.resetAngle();

        if (Status.competitionMode) {
            telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_COMP_UPDATE_INTERVAL_MS);
        } else {
            telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_UPDATE_INTERVAL_MS);
        }
        this.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
        // feedForward.accelTableInit();
    }

    public void start(OpMode opmode, boolean teleop) {
        gamepadEx1 = new GamepadWrapper(opmode.gamepad1);
        gamepadEx2 = new GamepadWrapper(opmode.gamepad2);
        Status.isDrivingActive = false;
        Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.RED ?
                        new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45) :
                        Status.alliance == Constants.Game.ALLIANCE.BLUE ?
                                new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, 45) :
                                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.goalsideStartingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GOALSIDE_STARTING_X, Constants.Robot.GOALSIDE_STARTING_Y, AngleUnit.DEGREES, Constants.Robot.GOALSIDE_STARTING_HEADING) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GOALSIDE_STARTING_X, -Constants.Robot.GOALSIDE_STARTING_Y, AngleUnit.DEGREES, Constants.Robot.GOALSIDE_STARTING_HEADING) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.startingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, Constants.Robot.STARTING_Y, AngleUnit.DEGREES, Constants.Robot.STARTING_HEADING) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, -Constants.Robot.STARTING_Y, AngleUnit.DEGREES, -Constants.Robot.STARTING_HEADING) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.cornerResetPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.CORNER_X, Constants.Robot.CORNER_Y, AngleUnit.DEGREES, Constants.Robot.CORNER_ANGLE) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.CORNER_X, -Constants.Robot.CORNER_Y, AngleUnit.DEGREES, -Constants.Robot.CORNER_ANGLE) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        RobotContainer.HardwareDevices.limelight.start(); // IDK what this does

        // Start the required threads
        telemetryLogger = new TelemetryLogger(this);
        telemetryLogger.start();
        localizationUpdater = new LocalizationUpdater(this);
        localizationUpdater.start();
        drivetrainUpdater = new DrivetrainUpdater(this);
        drivetrainUpdater.start();
        pathingUpdater = new PathingUpdater(this);
        if (!teleop) {
            pathingUpdater.start();
        } else {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        telemetryLoopTimer.reset();
    }

    public void stop() {
        Status.opModeIsActive = false;
        if (this.localizationUpdater != null) {
            this.localizationUpdater.stopThread();
            try {
                this.localizationUpdater.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (this.drivetrainUpdater != null) {
            this.drivetrainUpdater.stopThread();
            try {
                this.drivetrainUpdater.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (this.pathingUpdater != null) {
            this.pathingUpdater.stopThread();
            try {
                this.pathingUpdater.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (this.telemetryLogger != null) {
            this.telemetryLogger.stopThread();
            try {
                this.telemetryLogger.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void update(boolean teleop) {
        CURRENT_LOOP_TIME_MS = updateLoopTime("teleOp");
        refreshData();
        gamepadEx1.update();
        gamepadEx2.update();
        limelightLogic.update();
        delayedActionManager.update();

        allIndicatorLights.lightsUpdate();

        turret.update(teleop);
        spindexer.update(teleop);
        positionProvider.update(true);

        // Update the breamBreak state
        beamBreakToggleButton.update(HardwareDevices.beamBreak.isPressed());

        controlHubVoltage = getVoltage(Constants.Robot.CONTROL_HUB_INDEX);

        PREV_LOOP_TIME_MS = CURRENT_LOOP_TIME_MS;

        if (teleop) {
            telemetry("teleOp");
        }
    }

    /**
     * Retrieves a hardware device from the hardware map.
     * <p>
     * This method attempts to retrieve a specific hardware device from the hardware map,
     * based on the provided class type and device name. If the device is found, it is
     * returned; otherwise, an error message is added to the telemetry, and null is returned.
     *
     * @param <T>           The type of the hardware device being requested.
     * @param hardwareClass The class of the hardware device (e.g., DcMotor.class, Servo.class).
     * @param name          The name of the hardware device as configured in the Robot Controller app.
     * @return The requested hardware device if found; null otherwise.
     */
    public <T> T getHardwareDevice(Class<T> hardwareClass, String name) {
        try {
            return hardwareMap.get(hardwareClass, name);
        } catch (Exception e) {
            telemetry.addLine("Could not load hardware class: '" + name + "' and got error: '" + e + "'");
            return null; // Or throw the exception if you prefer
        }
    }

    /**
     * Add or update a retained line of telemetry.
     */
    public void addEventTelemetry(String caption, Object value) {
        eventTelemetryCaptions.add(caption);
        eventTelemetryValues.add(value);
        eventTelemetry.add("TIME OF EVENT" + ": " + (System.currentTimeMillis() - startTimeMs)+ "\n" + caption + ": " + value.toString());
    }

    public void displayEventTelemetry() {
        for (int i = 0; i < eventTelemetryCaptions.size(); i++) {
            telemetry.addData(eventTelemetryCaptions.get(i), eventTelemetryValues.get(i));
        }
    }

    /** This is for hardware error that are critical and code execution should stop to tell the user of the error. */
    public void testCriticalHardwareDevice(Object hardwareClass) {
        if (hardwareClass == null) {
            telemetry.log().clear();
            telemetry.addLine("Failed to load hardware class, class is null");
            telemetry.addLine("This message will show for 10 seconds.");
            telemetry.update();
            try {
                Thread.sleep(10000); // 10 seconds = 10000 milliseconds.
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Do something here, prob something else.
            }
        }
    }

    /**
     * Refreshes the data from all Lynx Modules (hubs) by clearing their bulk data cache.
     * <p>
     * This method is crucial for ensuring that the robot is operating with the most up-to-date
     * sensor and motor data. The Lynx Modules use a bulk data cache to optimize data transfer.
     * However, if data in this cache becomes stale, the robot's actions might be based on
     * outdated information.
     * <p>
     * This method should be called periodically or whenever you suspect that the data in the
     * bulk cache might be outdated. Common scenarios include:
     * - At the start of a new control loop iteration in teleop or autonomous.
     * - After a significant delay or pause in the robot's operation.
     * - Before reading critical sensor values that need to be absolutely current.
     * - If there is a change in the bulk caching mode.
     * <p>
     * Calling this method ensures that the next time you read data from the hubs, the latest
     * information will be fetched, rather than possibly outdated cached data.
     */
    public void refreshData() {
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getVoltage(int hubIndex) {
        LynxModule selectedHub;

        // Determine which hub to use based on hubIndex
        switch (hubIndex) {
            case 0:
                selectedHub = HardwareDevices.controlHub;
                break;
            case 1:
                selectedHub = HardwareDevices.expansionHub;
                break;
            default:
                // Invalid index
                opMode.telemetry.addLine("ERROR: Invalid hub index");
                opMode.telemetry.update();
                return -1; // Or throw an exception
        }

        if (selectedHub == null) {
            opMode.telemetry.addLine("ERROR: Hub not found");
            opMode.telemetry.update();
            return -1;
        }

        return selectedHub.getInputVoltage(VoltageUnit.VOLTS);
    }

    public double getCurrent(int hubIndex) {
        LynxModule selectedHub;

        // Determine which hub to use based on hubIndex
        switch (hubIndex) {
            case 0:
                selectedHub = HardwareDevices.controlHub;
                break;
            case 1:
                selectedHub = HardwareDevices.expansionHub;
                break;
            default:
                // Invalid index
                addEventTelemetry("ERROR", "Invalid hub index");
                return -1; // Or throw an exception
        }

        if (selectedHub == null) {
            addEventTelemetry("ERROR", "Hub not found");
            return -1;
        }

        return selectedHub.getCurrent(CurrentUnit.AMPS);
    }

    public void registerLoopTimer(String name) {
        loopTimesMap.put(name, new LinkedList<>());
        loopTimers.put(name, new ElapsedTime());
    }

    /**
     * Call at the top of your named loop.
     * @return The elapsed time (ms) since last call for this name.
     */
    public double updateLoopTime(String name) {
        ElapsedTime timer = loopTimers.get(name);
        LinkedList<Double> times = loopTimesMap.get(name);
        if (timer == null || times == null) {
            throw new IllegalArgumentException("LoopTimer not registered: " + name);
        }
        double dt = timer.milliseconds();
        timer.reset();
        times.add(dt);
        if (times.size() > Constants.System.LOOP_AVERAGE_WINDOW_SIZE) {
            times.removeFirst();
        }
        return dt;
    }

    /** Call anywhere to get the rolling average for that loop name. */
    public double getRollingAverageLoopTime(String name) {
        LinkedList<Double> times = loopTimesMap.get(name);
        if (times == null || times.isEmpty()) return 0;
        double sum = 0;
        for (double t : times) sum += t;
        return sum / times.size();
    }

    public double getLoopTime(String name) {
        LinkedList<Double> times = loopTimesMap.get(name);
        if (times == null || times.isEmpty()) return 0;
        return times.getLast();
    }

    public void writeToFile (String fileName, String data) {
        File myFileName = AppUtil.getInstance().getSettingsFile(fileName);
        ReadWriteFile.writeFile(myFileName, data);
    }

    public void addDataLog(String caption, Object data, boolean driveStation) {
        if (driveStation) {
            telemetry.addData(caption, data);
        }

        if (!Status.loggingToFile) {
            return;
        }

        if (data == null) data = "null";

        String dataString = data.toString();

        dataString = dataString.replaceAll(",", "|");


        // Add new headers if needed
        if (!telemetryHeaderList.contains(caption)) {
            telemetryHeaderList.add(caption);
            telemetryCache.put(caption, new ArrayList<>());
        }

        // Put this loop’s value in the buffer
        currentLoopData.put(caption, dataString);
    }

    public void commitLoopData() {
        if (!Status.loggingToFile) {
            return;
        }
        // Determine the current row number
        int row = telemetryCache.get(telemetryHeaderList.get(0)).size();

        // For every column, add value from buffer or empty string
        for (String header : telemetryHeaderList) {
            ArrayList<String> column = telemetryCache.get(header);
            String value = currentLoopData.getOrDefault(header, "").toString();
            column.add(value);
        }

        // Clear the buffer for the next loop
        currentLoopData.clear();
    }

    public void writeDataLog() {
        StringBuilder csvLog = new StringBuilder();

        for (int i = 0; i < telemetryHeaderList.size(); i++) {
            csvLog.append(telemetryHeaderList.get(i));
            if (i < telemetryHeaderList.size() - 1) csvLog.append(',');
        }
        csvLog.append('\n');

        int rowCount = 0;
        if (!telemetryHeaderList.isEmpty()) {
            rowCount = telemetryCache.get(telemetryHeaderList.get(0)).size();
        }

        for (int row = 0; row < rowCount; row++) {
            for (int col = 0; col < telemetryHeaderList.size(); col++) {
                String header = telemetryHeaderList.get(col);
                ArrayList<String> columnData = telemetryCache.get(header);
                csvLog.append(columnData.get(row));
                if (col < telemetryHeaderList.size() - 1) csvLog.append(',');
            }
            csvLog.append('\n');
        }

        writeToFile("TelemetryLog.txt", csvLog.toString());
    }

    public void writeEventLog() {
        StringBuilder log = new StringBuilder();

        for (int i = 0; i < eventTelemetry.size(); i++) {
            log.append(eventTelemetry.get(i));
            log.append('\n');
            log.append('\n');
        }

        writeToFile("EventLog.txt", log.toString());
    }

    public void telemetry(String opMode) {
        if (telemetryLoopTimer.milliseconds() < Constants.System.TELEMETRY_UPDATE_INTERVAL_MS && !Status.competitionMode) {
            return;
        } else if (telemetryLoopTimer.milliseconds() < Constants.System.TELEMETRY_COMP_UPDATE_INTERVAL_MS && Status.competitionMode) {
            return;
        }

        // This line is required for the logViwer to work correctly
        // It also need to be at column zero and spelled exactly "Time Stamp"
        addDataLog("Time Stamp", System.currentTimeMillis() - startTimeMs, true);

        // Stuff also in competition mode
        addDataLog("Alliance", Status.alliance, true);
        telemetry.addLine();
        addDataLog("Spindexer Slot Colors", spindexer.slotColor[0].toString() + spindexer.slotColor[1].toString() + spindexer.slotColor[2].toString(), true);
        telemetry.addLine();
        addDataLog("OpMode Avg Loop Time", (int) getRollingAverageLoopTime(opMode) + " ms", true);
        addDataLog("DriveTrain Avg Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms", true);
        addDataLog("Pinpoint Avg Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms", true);
        telemetry.addLine();
        addDataLog("Pinpoint X", Status.currentPose.getX(DistanceUnit.CM) + " cm", true);
        addDataLog("Pinpoint Y", Status.currentPose.getY(DistanceUnit.CM) + " cm", true);
        addDataLog("Robot Heading", Status.currentHeading + "°", true);
        addDataLog("Logging to file", Status.loggingToFile, true);

        addDataLog("BetterIMU Yaw", HardwareDevices.betterIMU.getAngle(), true); // First angle is the yaw
        addDataLog("Pinpoint Yaw", HardwareDevices.pinpoint.getHeading(AngleUnit.DEGREES), true); // First angle is the yaw
        addDataLog("Use Better IMU", Constants.USE_BETTER_IMU, true);

        if (!Status.competitionMode) {

            // Stuff not in competition mode.
            
            // Get current and voltage for telemetry
            controlHubVoltage = getVoltage(Constants.Robot.CONTROL_HUB_INDEX);
            expansionHubVoltage = getVoltage(Constants.Robot.EXPANSION_HUB_INDEX);
            controlHubCurrent = getCurrent(Constants.Robot.CONTROL_HUB_INDEX);
            expansionHubCurrent = getCurrent(Constants.Robot.EXPANSION_HUB_INDEX);
            addDataLog("Control Hub Voltage", controlHubVoltage + " V", true);
            addDataLog("Expansion Hub Voltage", expansionHubVoltage + " V", true);
            addDataLog("Control Hub Current", controlHubCurrent + " A", true);
            addDataLog("Expansion Hub Current", expansionHubCurrent + " A", true);
            addDataLog("Total Amps", controlHubCurrent + expansionHubCurrent + " A", true);
            telemetry.addLine();
            addDataLog("Spindexer Angle", spindexer.getAngle(), true);
            addDataLog("Spindexer Intake Slot", spindexer.getCurrentIntakeSlot(), true);
            addDataLog("Spindexer Target Angle", spindexer.targetAngle, true);
            addDataLog("Spindexer Error Angle", spindexer.getError(), true);
            addDataLog("Flywheel Target Velocity", turret.flywheel.targetVelocity, true);
            addDataLog("Flywheel Current Velocity", HardwareDevices.flyWheelMotorMaster.getVelocity(), true);
            addDataLog("Flywheel Main Motor Current mA", HardwareDevices.flyWheelMotorMaster.getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Flywheel Secondary Motor Current mA", HardwareDevices.flyWheelMotorSlave.getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Flywheel2 Current Velocity", HardwareDevices.flyWheelMotorSlave.getVelocity(), true);
            telemetry.addLine();
            if (limelightLogic.limelight.getLatestResult() != null) {
                addDataLog("Limelight Current Visibility", true, true);
                addDataLog("Limelight Current Result", limelightLogic.hasResult(), true);
            } else {
                addDataLog("Limelight Current Visibility", false, true);
            }
            LimeLightInfo LLInfo = limelightLogic.logicBotPoseCM();
            if (LLInfo != null) {
                addDataLog("Limelight tx", LLInfo.result.getTx(), true);
                addDataLog("Limelight ty", LLInfo.result.getTy(), true);
            } else {
                addDataLog("Limelight tx", null, true);
                addDataLog("Limelight ty", null, true);
            }

            addDataLog("Turret Current Angle", turret.getPositionDegrees(), true);
            addDataLog("Hood Position", HardwareDevices.hoodServo.getPosition(), true);
            telemetry.addLine();
            addDataLog("Limelight Offset Position", positionProvider.getVisionOffsetPose(), true);
            telemetry.addLine();
            addDataLog("Robot Heading", Status.currentHeading + "°", true);
            addDataLog("Pinpoint Heading", RobotContainer.HardwareDevices.pinpoint.getPosition().getHeading(AngleUnit.DEGREES), true);
            addDataLog("Pinpoint Status", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus(), true);
            addDataLog("Distance to Goal", HelperFunctions.disToGoal(), true);
            telemetry.addLine();
            addDataLog("OpMode Loop Time", CURRENT_LOOP_TIME_MS + " ms", true);
            addDataLog("DriveTrain Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_MS + " ms", true);
            addDataLog("Pinpoint Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_MS + " ms", true);
            telemetry.addLine();
            addDataLog("Goal Position", Status.goalPose, true);
            addDataLog("Start Position", Status.startingPose, true);
            telemetry.addLine();
            addDataLog("Field Oriented", Status.fieldOriented, true);
            addDataLog("Intake Toggle", Status.intakeGamepadable, true);
            telemetry.addLine();
            addDataLog("Beam Break", HardwareDevices.beamBreak.isPressed(), true);
            // addDataLog("Red", HardwareDevices.colorSensor.updateRed(), true);
            // addDataLog("Blue", HardwareDevices.colorSensor.updateBlue(), true);
            // addDataLog("Green", HardwareDevices.colorSensor.updateGreen(), true);
            // addDataLog("Color Sensor Distance (cm)", HardwareDevices.colorSensor.getDistance(), true);
            //
            // for (int i = 0; i < swerveModules.length; i++) {
            //     telemetry.addLine();
            //     telemetry.addLine("Servo" + i);
            //     addDataLog("Swerve Servo " + i + " Angle", swerveModules[i].servo.getAngle(), true);
            //     addDataLog("Swerve Servo " + i + " Target", swerveServosPDF[i].getTargetAngle(), true);
            //     addDataLog("Swerve Servo " + i + " Servo Target Power", swerveServosPDF[i].calculate(), true);
            //     addDataLog("Swerve Servo " + i + " Servo Error", swerveServosPDF[i].getError(), true);
            //     addDataLog("Swerve Motor " + i + " Target Power", swerveModules[i].motor.targetPower, true);
            //     addDataLog("Swerve Motor " + i + " Current Velocity", swerveModules[i].motor.getVelocity(), true);
            //     addDataLog("Swerve Motor " + i + " Current Power", RobotContainer.HardwareDevices.swerveMotors[i].getPower(), true);
            // }
            //

            // Test Power draw
            addDataLog("Motor Intake", HardwareDevices.intakeMotor.getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Flywheel 1", HardwareDevices.flyWheelMotorMaster.getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Flywheel 2", HardwareDevices.flyWheelMotorSlave.getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Swerve 1", HardwareDevices.swerveMotors[0].getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Swerve 2", HardwareDevices.swerveMotors[1].getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Swerve 3", HardwareDevices.swerveMotors[2].getCurrent(CurrentUnit.MILLIAMPS), true);
            addDataLog("Motor Swerve 4", HardwareDevices.swerveMotors[3].getCurrent(CurrentUnit.MILLIAMPS), true);

            addDataLog("Servo Hood", HardwareDevices.hoodServo.getPosition(), true);
            addDataLog("Servo Turret 1", HardwareDevices.turretServoMaster.getPosition(), true);
            addDataLog("Servo Turret 2", HardwareDevices.turretServoSlave.getPosition(), true);
            addDataLog("Servo Swerve 1", HardwareDevices.swerveServos[0].getPosition(), true);
            addDataLog("Servo Swerve 2", HardwareDevices.swerveServos[1].getPosition(), true);
            addDataLog("Servo Swerve 3", HardwareDevices.swerveServos[2].getPosition(), true);
            addDataLog("Servo Swerve 4", HardwareDevices.swerveServos[3].getPosition(), true);
            addDataLog("Servo Spindexer 1", HardwareDevices.spindexerServoMaster.getPower(), true);
            addDataLog("Servo Spindexer 2", HardwareDevices.spindexerServoSlave.getPower(), true);

            telemetry.addLine();
            displayEventTelemetry();
            commitLoopData();
        }
        telemetry.update();
    }
}
