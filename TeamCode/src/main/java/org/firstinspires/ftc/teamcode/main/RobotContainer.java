package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathingUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.HeadingPID;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.drivetrain.SwervePDF;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LatitudePID;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LongitudePID;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensLogic;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLogic;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.other.IndicatorLighting;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.subsystems.PositionProvider;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

import org.firstinspires.ftc.teamcode.hardware.BetterTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;

import java.lang.Thread;
import java.util.ArrayList;
import java.util.Arrays;
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
    private final ArrayList<String> retainedTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> retainedTelemetryValues = new ArrayList<>();
    public final SwerveModule[] swerveModules = new SwerveModule[Constants.Swerve.NUM_MOTORS];
    public SwervePDF[] swerveServosPDF = new SwervePDF[Constants.Swerve.NUM_SERVOS];
    public LocalizationUpdater localizationUpdater;
    public DrivetrainUpdater drivetrainUpdater;
    public PathingUpdater pathingUpdater;
    public PathPlanner pathPlanner;
    public LimelightLogic limelightLogic;
    public PositionProvider positionProvider;
    public HuskyLensLogic huskyLensLogic1;
    public HuskyLensLogic huskyLensLogic2;
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
    public Transfer transfer;

    public double controlHubVoltage;
    public double expansionHubVoltage;
    public double controlHubCurrent;
    public double expansionHubCurrent;

    public double CURRENT_LOOP_TIME_MS;
    public double PREV_LOOP_TIME_MS;
    public double DELTA_TIME_MS;

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static LynxModule controlHub;
        public static LynxModule expansionHub;
        public static IMU imu;

        public static GoBildaPinpointDriver pinpoint;
        public static Limelight3A limelight;
        public static HuskyLens huskyLens1;
        public static HuskyLens huskyLens2;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLightFront;
        public static ServoImplEx indicatorLightBack;

        // Gobilda 5000 Series
        public static BetterDcMotor[] swerveMotors = new BetterDcMotor[Constants.Swerve.NUM_MOTORS];
            public static String[] motorNames = new String[Constants.Swerve.NUM_MOTORS];

        // Axon Mini+
        public static BetterCRServo[] swerveServos = new BetterCRServo[Constants.Swerve.NUM_SERVOS];
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
        public static BetterServo transferServoLeft;
        public static BetterServo transferServoRight;
        public static BetterDcMotor spindexerEncoder;

        public static BetterCRServo spindexServo;
        public static BetterAnalogInput spindexAnalog;

        // Intake
        public static BetterDcMotor intakeMotor;

        //sensors
        public static BetterColorSensor colorSensor;
        public static RevTouchSensor beamBreak;
    }

    public RobotContainer(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        HardwareDevices.imu = getHardwareDevice(IMU.class, "imu");
        HardwareDevices.imu.initialize(Constants.Robot.imuParameters);

        HardwareDevices.pinpoint = getHardwareDevice(GoBildaPinpointDriver.class, "pinpoint");
        HardwareDevices.pinpoint.setOffsets(Constants.Pathing.PINPOINT_X_OFFSET_MM, Constants.Pathing.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        HardwareDevices.pinpoint.setEncoderResolution(Constants.Pathing.PINPOINT_ODOM_POD);
        HardwareDevices.pinpoint.setEncoderDirections(Constants.Pathing.PINPOINT_X_ENCODER_DIRECTION, Constants.Pathing.PINPOINT_Y_ENCODER_DIRECTION);

        HardwareDevices.limelight = getHardwareDevice(Limelight3A.class, "limelight");
        HardwareDevices.huskyLens1 = getHardwareDevice(HuskyLens.class, "huskyLens1");
        HardwareDevices.huskyLens2 = getHardwareDevice(HuskyLens.class, "huskyLens2");

        HardwareDevices.indicatorLightFront = getHardwareDevice(ServoImplEx.class, "indicatorLightFront");
        HardwareDevices.indicatorLightBack = getHardwareDevice(ServoImplEx.class, "indicatorLightBack");

        for (int i = 0; i < swerveModules.length; i++) {
            HardwareDevices.motorNames[i] = "swerveMotor" + (i);
            HardwareDevices.swerveMotors[i] = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, HardwareDevices.motorNames[i]), Constants.Robot.SWERVE_MOTOR_UPDATE_TIME);
            HardwareDevices.servoNames[i] = "swerveServo" + (i);
            HardwareDevices.swerveServos[i] = new BetterCRServo(hardwareMap.get(CRServoImplEx.class, HardwareDevices.servoNames[i]), Constants.Robot.SWERVE_SERVO_UPDATE_TIME);
            HardwareDevices.analogNames[i] = "swerveAnalog" + (i);
            HardwareDevices.swerveAnalogs[i] = new BetterAnalogInput(hardwareMap.get(AnalogInput.class, HardwareDevices.analogNames[i]), Constants.Robot.SWERVE_ANALOG_UPDATE_TIME);

            if (i == 0 || i == 2) {
                HardwareDevices.swerveMotors[i].setDirection(DcMotorImplEx.Direction.REVERSE);
            }
            HardwareDevices.swerveMotors[i].setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
            HardwareDevices.swerveMotors[i].setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

            swerveServosPDF[i] = new SwervePDF(this, i);
            swerveModules[i] = new SwerveModule(this, HardwareDevices.swerveMotors[i], HardwareDevices.swerveServos[i], swerveServosPDF[i], HardwareDevices.swerveAnalogs[i], Constants.Swerve.POWER_MULTIPLIER[i], i);  //is it best to pass in a constant?

            if (Constants.Swerve.SERVO_ANALOG_ACTIVE) {
                int analogPortNumber = Character.getNumericValue(HardwareDevices.swerveAnalogs[i].getConnectionInfo().charAt(HardwareDevices.swerveAnalogs[i].getConnectionInfo().length() - 1));
                if (analogPortNumber != i) {
                    addRetainedTelemetry("WARNING: Swerve Analog Encoder " + i + " is connected to port " + analogPortNumber + ", should be port " + i, null);
                }
            }
        }

        HardwareDevices.turretServoMaster = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoMaster"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.turretServoSlave = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoSlave"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.hoodServo = new BetterServo(getHardwareDevice(ServoImplEx.class, "hoodServo"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.transferServoLeft = new BetterServo(getHardwareDevice(ServoImplEx.class, "transferServoLeft"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.transferServoRight = new BetterServo(getHardwareDevice(ServoImplEx.class, "transferServoRight"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.spindexerEncoder = new BetterDcMotor(getHardwareDevice(DcMotorImplEx.class, "spindexEncoder"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.spindexServo = new BetterCRServo(getHardwareDevice(CRServoImplEx.class, "spindexServo"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.spindexAnalog = new BetterAnalogInput(getHardwareDevice(AnalogInput.class, "spindexAnalog"), Constants.Robot.ANALOG_UPDATE_TIME);
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

        pathPlanner = new PathPlanner(telemetry);
        Arrays.fill(Status.pathCompleted, false);

        limelightLogic = new LimelightLogic(this, telemetry, HardwareDevices.limelight);
        positionProvider = new PositionProvider(this, limelightLogic, HardwareDevices.pinpoint);
        huskyLensLogic1 = new HuskyLensLogic(this, RobotContainer.HardwareDevices.huskyLens1);
        huskyLensLogic2 = new HuskyLensLogic(this, RobotContainer.HardwareDevices.huskyLens2);
        turret = new Turret(this, flyWheelMotors, HardwareDevices.hoodServo, turretServos);
        HardwareDevices.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(this, HardwareDevices.intakeMotor);
        spindexer = new Spindexer(this, HardwareDevices.spindexServo, HardwareDevices.spindexAnalog, HardwareDevices.colorSensor);
        transfer = new Transfer(this, HardwareDevices.transferServoLeft, HardwareDevices.transferServoRight);

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
    }

    public void init() {
        this.isRunning = true;
        RobotContainer.HardwareDevices.imu.resetYaw();
        this.pathPlanner = new PathPlanner(this.telemetry);
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        HardwareDevices.controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        HardwareDevices.expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_UPDATE_INTERVAL_MS);
        this.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
        transfer.flapDown();
    }

    public void start(OpMode opmode, boolean teleop) {
        gamepadEx1 = new GamepadWrapper(opmode.gamepad1);
        gamepadEx2 = new GamepadWrapper(opmode.gamepad2);
        Status.isDrivingActive = false;
        Status.GOAL_POSE = Status.alliance == Constants.Game.ALLIANCE.RED ?
                        new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45) :
                        Status.alliance == Constants.Game.ALLIANCE.BLUE ?
                                new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, 45) :
                                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.goalsideStartingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GoalsideStartingX, Constants.Robot.GoalsideStartingY, AngleUnit.DEGREES, Constants.Robot.GoalsideStartingHeading) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GoalsideStartingX, -Constants.Robot.GoalsideStartingY, AngleUnit.DEGREES, Constants.Robot.GoalsideStartingHeading) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.startingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.startingX, Constants.Robot.startingY, AngleUnit.DEGREES, Constants.Robot.startingHeading) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.startingX, -Constants.Robot.startingY, AngleUnit.DEGREES, Constants.Robot.startingHeading) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        // Start the required treads
        localizationUpdater = new LocalizationUpdater(this);
        localizationUpdater.start();
        drivetrainUpdater = new DrivetrainUpdater(this);
        drivetrainUpdater.start();
        pathingUpdater = new PathingUpdater(this);
        if (!teleop) {
            pathingUpdater.start();
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
    public void addRetainedTelemetry(String caption, Object value) {
        retainedTelemetryCaptions.add(caption);
        retainedTelemetryValues.add(value);
    }

    public void displayRetainedTelemetry() {
        for (int i = 0; i < retainedTelemetryCaptions.size(); i++) {
            telemetry.addData(retainedTelemetryCaptions.get(i), retainedTelemetryValues.get(i));
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
                addRetainedTelemetry("ERROR", "Invalid hub index");
                return -1; // Or throw an exception
        }

        if (selectedHub == null) {
            addRetainedTelemetry("ERROR", "Hub not found");
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

    public void telemetry (String opMode) {
        if (telemetryLoopTimer.milliseconds() < Constants.System.TELEMETRY_UPDATE_INTERVAL_MS) {
            return;
        }
        if (Status.competitionMode) {
            telemetry.addData("Spindexer Slot Colors", Arrays.toString(Status.slotColor));
            return;
        }
        //TelemetryPacket packet = new TelemetryPacket();
//        telemetry.addData("Control Hub Voltage", controlHubVoltage + " V");
//        telemetry.addData("Expansion Hub Voltage", expansionHubVoltage + " V");
//        telemetry.addData("Control Hub Current", controlHubCurrent + " A");
//        telemetry.addData("Expansion Hub Current", expansionHubCurrent + " A");
        telemetry.addLine();
        telemetry.addData("Spindexer Angle", spindexer.getAngle());
        telemetry.addData("Spindexer Intake Slot", spindexer.getCurrentIntakeSlot());
        telemetry.addData("Spindexer Target Angle", spindexer.targetAngle);
        telemetry.addData("Spindexer Error Angle", spindexer.getError());
        telemetry.addData("Spindexer Slot Colors", Arrays.toString(Status.slotColor));
        telemetry.addData("flywheel target max vel", turret.flywheel.targetVelocity);
        telemetry.addData("flywheel speed", HardwareDevices.flyWheelMotorMaster.getVelocity());
        telemetry.addData("flywheel atVelocity", turret.flywheel.atTargetVelocity());
         telemetry.addData("turret interpolation", turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
        telemetry.addLine();
        telemetry.addData("Vision Pose List Size", positionProvider.getVisionPoseList().size());

        if (limelightLogic.limelight.getLatestResult() != null) {
            telemetry.addData("LL SEE", "yay");
        } else {
            telemetry.addData("LL IS BLIND", "no yay");
        }
        LimeLightInfo LLInfo = limelightLogic.limelightInfo();
        if (LLInfo != null) {
            telemetry.addData("Vision tx", limelightLogic.limelightInfo().result.getTx());
            telemetry.addData("Vision ty", limelightLogic.limelightInfo().result.getTy());
        } else {
            telemetry.addData("Vision tx", 0);
            telemetry.addData("Vision ty", 0);
        }

        telemetry.addData("robot pos on field CM", positionProvider.getRobotPose());
        telemetry.addLine();
        telemetry.addData("Vision offset pose", positionProvider.getVisionOffsetPose());
        telemetry.addLine();
        telemetry.addData("Pinpoint position", HardwareDevices.pinpoint.getPosition());
//        telemetry.addData("Pinpoint X", Status.currentPose.getX(DistanceUnit.CM) + " cm");
//        telemetry.addData("Pinpoint Y", Status.currentPose.getY(DistanceUnit.CM) + " cm");
//        telemetry.addData("Pinpoint Heading", Status.currentHeading + "°");
        telemetry.addData("PINPOINT STATUS", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        telemetry.addData("odo x", RobotContainer.HardwareDevices.pinpoint.getEncoderX());
        telemetry.addData("odo y", RobotContainer.HardwareDevices.pinpoint.getEncoderY());
        telemetry.addData("PINPOINT STATUS", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        telemetry.addData("Dist to goal", HelperFunctions.disToGoal());
        telemetry.addLine();
        telemetry.addData("OpMode Avg Loop Time", (int) getRollingAverageLoopTime(opMode) + " ms");
        telemetry.addData("OpMode Loop Time", CURRENT_LOOP_TIME_MS + " ms");
        telemetry.addData("OpMode Delta Time", DELTA_TIME_MS + " ms");
        telemetry.addLine();
        telemetry.addData("DriveTrain Avg Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        telemetry.addData("DriveTrain Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_MS + " ms");
        telemetry.addData("Goal: ", Status.GOAL_POSE);
        telemetry.addData("Start: ", Status.startingPose);
//        telemetry.addLine();
//        telemetry.addData("Pinpoint Avg Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
//        telemetry.addData("Pinpoint Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_MS + " ms");
//        telemetry.addLine();
//        telemetry.addData("Heading PID Target", headingPID.getTargetHeading());
//        telemetry.addData("Heading PID Target Reached", Status.robotHeadingTargetReached);
//        telemetry.addData("Heading PID Output", headingPID.calculate(Status.currentHeading));
//        telemetry.addLine();
//        telemetry.addData("Left Stick Y", gamepadEx1.leftStickY());
//        telemetry.addData("Left Stick X", gamepadEx1.leftStickX());
//        telemetry.addData("Right Stick Y", gamepadEx1.rightStickY());
//        telemetry.addData("Right Stick X", gamepadEx1.rightStickX());
//        telemetry.addLine();
//        telemetry.addData("Motor 0 Current Velocity", swerveModules[0].motor.getVelocity());
//        telemetry.addData("Motor 1 Current Velocity", swerveModules[1].motor.getVelocity());
//        telemetry.addData("Motor 2 Current Velocity", swerveModules[2].motor.getVelocity());
//        telemetry.addData("Motor 3 Current Velocity", swerveModules[3].motor.getVelocity());
        telemetry.addData("Field Oriented", Status.fieldOriented);
        telemetry.addData("Intake Enabled", Status.intakeToggle);
        telemetry.addLine();
        telemetry.addData("lower servo pos", HardwareDevices.transferServoLeft.getPosition());
        // telemetry.addData("flywheel current mA", HardwareDevices.flyWheelMotorMaster.getCurrent(CurrentUnit.MILLIAMPS));
        // telemetry.addData("upper flywheel current mA", HardwareDevices.flyWheelMotorSlave.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("turret pos", turret.getPosition());
        telemetry.addData("slave servo", HardwareDevices.turretServoSlave.getPosition());
        telemetry.addData("hood", HardwareDevices.hoodServo.getPosition());
        telemetry.addLine();
        telemetry.addData("beam break", HardwareDevices.beamBreak.isPressed());
//       telemetry.addData("red", HardwareDevices.colorSensor.updateRed());
//       telemetry.addData("blue", HardwareDevices.colorSensor.updateBlue());
//       telemetry.addData("green", HardwareDevices.colorSensor.updateGreen());
//       telemetry.addData("color sensor dist", HardwareDevices.colorSensor.getDistance());

//        int selectedServo = -1;
//        if (gamepadEx1.dpadUp.isPressed()) {
//            selectedServo = 0;
//        } else if (gamepadEx1.dpadRight.isPressed()) {
//            selectedServo = 1;
//        } else if (gamepadEx1.dpadDown.isPressed()) {
//            selectedServo = 2;
//        } else if (gamepadEx1.dpadLeft.isPressed()) {
//            selectedServo = 3;
//        }
//        if (selectedServo >= 0) {
//            telemetry.addLine();
//            telemetry.addData("Selected Servo", selectedServo);
//            telemetry.addData("Servo Angle", swerveModules[selectedServo].servo.getAngle());
//            telemetry.addData("Servo Target", swerveServosPDF[selectedServo].getTargetAngle());
//            telemetry.addData("Servo Set Power", swerveServosPDF[selectedServo].calculate());
//            telemetry.addData("Servo Error", swerveServosPDF[selectedServo].getError());
//            telemetry.addData("Motor Target Power", swerveModules[selectedServo].motor.targetPower);
//            telemetry.addData("Motor Current Velocity", swerveModules[selectedServo].motor.getVelocity());
//            telemetry.addData("Motor Current Power", RobotContainer.HardwareDevices.swerveMotors[selectedServo].getPower());
//        }
//        packet.fieldOverlay()
//                .drawImage("teamcode/other/DECODE_FIELD.png", 24, 24, 48, 48);
        telemetry.addLine();
        displayRetainedTelemetry();
        telemetry.update();
    }
}
