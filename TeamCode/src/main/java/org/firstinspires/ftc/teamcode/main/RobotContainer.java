package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.HeadingPID;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.drivetrain.SwervePIDF;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensLogic;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLogic;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.other.ADG728;
import org.firstinspires.ftc.teamcode.other.ADGUpdater;
import org.firstinspires.ftc.teamcode.other.IndicatorLighting;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

import java.lang.Thread;
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
    public OpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public boolean isRunning = false;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
    public final Map<String, LinkedList<Double>> loopTimesMap = new HashMap<>();
    public final Map<String, ElapsedTime> loopTimers = new HashMap<>();
    private final ElapsedTime telemetryLoopTimer = new ElapsedTime();
    private final ElapsedTime timer1 = new ElapsedTime();
    private final ElapsedTime timer2 = new ElapsedTime();
    private final ArrayList<String> retainedTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> retainedTelemetryValues = new ArrayList<>();
    public final SwerveModule[] swerveModules = new SwerveModule[Constants.Swerve.NUM_MOTORS];
    public SwervePIDF[] swerveServosPIDF = new SwervePIDF[Constants.Swerve.NUM_SERVOS];
    public LocalizationUpdater localizationUpdater;
    public DrivetrainUpdater drivetrainUpdater;
    public ADGUpdater adgUpdater;
    public PathPlanner pathPlanner;
    public LimelightLogic limelightLogic;
    public HuskyLensLogic huskyLensLogic1;
    public HuskyLensLogic huskyLensLogic2;
    public DelayedActionManager delayedActionManager = new DelayedActionManager();
    public Drivetrain drivetrain;
    public HeadingPID headingPID;
    public IndicatorLighting.Light indicatorLightFront;
    public IndicatorLighting.Light indicatorLightBack;
    public IndicatorLighting.Group allIndicatorLights;
    public Turret turret;
    public Intake intake;
    public Spindexer spindexer;
    public Transfer transfer;

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static LynxModule controlHub;
        public static LynxModule expansionHub;
        public static IMU imu;

        public static GoBildaPinpointDriver pinpoint;
        public static Limelight3A limelight;
        public static HuskyLens huskyLens1;
        public static HuskyLens huskyLens2;

        // This is the I2C multiplexer
        public static ADG728 mux1;
        public static AnalogInput muxAnalog1;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLightFront;
        public static ServoImplEx indicatorLightBack;

        // Gobilda 5000 Series
        public static DcMotorImplEx[] swerveMotors = new DcMotorImplEx[Constants.Swerve.NUM_MOTORS];
            public static String[] motorNames = new String[Constants.Swerve.NUM_MOTORS];

        // Axon Mini+
        public static CRServoImplEx[] swerveServos = new CRServoImplEx[Constants.Swerve.NUM_SERVOS];
            public static String[] servoNames = new String[Constants.Swerve.NUM_SERVOS];

        public static AnalogInput[] swerveAnalogs = new AnalogInput[Constants.Swerve.NUM_ANALOGS];
            public static String[] analogNames = new String[Constants.Swerve.NUM_ANALOGS];

        // Turret
        public static DcMotorImplEx flyWheelMotorMaster;
        public static DcMotorImplEx flyWheelMotorSlave;
        public static ServoImplEx turretServoMaster;
        public static ServoImplEx turretServoSlave;
        public static ServoImplEx hoodServo;

        // Spindexer
        public static CRServoImplEx transferServoLow;
        public static CRServoImplEx transferServoHigh;

        public static CRServoImplEx spindexServo;
        public static AnalogInput spindexAnalog;

        // Intake
        public static DcMotorImplEx intakeMotor;

        //sensors
        public static RevColorSensorV3 colorSensor;
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
        HardwareDevices.pinpoint.setPosition(Constants.Robot.startingPose);

        HardwareDevices.limelight = getHardwareDevice(Limelight3A.class, "limelight");
        HardwareDevices.huskyLens1 = getHardwareDevice(HuskyLens.class, "huskyLens1");
        HardwareDevices.huskyLens2 = getHardwareDevice(HuskyLens.class, "huskyLens2");

        HardwareDevices.mux1 = getHardwareDevice(ADG728.class, "mux");
        HardwareDevices.muxAnalog1 = getHardwareDevice(AnalogInput.class, "muxA1");
//        HardwareDevices.mux1.attachAnalog(HardwareDevices.muxAnalog1);

        HardwareDevices.indicatorLightFront = getHardwareDevice(ServoImplEx.class, "indicatorLightFront");
        HardwareDevices.indicatorLightBack = getHardwareDevice(ServoImplEx.class, "indicatorLightBack");

        for (int i = 0; i < swerveModules.length; i++) {
            HardwareDevices.motorNames[i] = "swerveMotor" + (i);
            HardwareDevices.swerveMotors[i] = getHardwareDevice(DcMotorImplEx.class, HardwareDevices.motorNames[i]);
            HardwareDevices.servoNames[i] = "swerveServo" + (i);
            HardwareDevices.swerveServos[i] = getHardwareDevice(CRServoImplEx.class, HardwareDevices.servoNames[i]);
            HardwareDevices.analogNames[i] = "swerveAnalog" + (i);
            HardwareDevices.swerveAnalogs[i] = getHardwareDevice(AnalogInput.class, HardwareDevices.analogNames[i]);

            if (i == 0 || i == 2) {
                HardwareDevices.swerveMotors[i].setDirection(DcMotorImplEx.Direction.REVERSE);
            }
            HardwareDevices.swerveMotors[i].setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
            HardwareDevices.swerveMotors[i].setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

            swerveServosPIDF[i] = new SwervePIDF(this, i);
            swerveModules[i] = new SwerveModule(this, HardwareDevices.swerveMotors[i], HardwareDevices.swerveServos[i], swerveServosPIDF[i], HardwareDevices.swerveAnalogs[i], Constants.Swerve.POWER_MULTIPLIER[i], i);  //is it best to pass in a constant?

            if (Constants.Swerve.SERVO_ANALOG_ACTIVE) {
                int analogPortNumber = Character.getNumericValue(HardwareDevices.swerveAnalogs[i].getConnectionInfo().charAt(HardwareDevices.swerveAnalogs[i].getConnectionInfo().length() - 1));
                if (analogPortNumber != i) {
                    addRetainedTelemetry("WARNING: Swerve Analog Encoder " + i + " is connected to port " + analogPortNumber + ", should be port " + i, null);
                }
            }
            if (HardwareDevices.swerveMotors[i].getPortNumber() != i) {
                addRetainedTelemetry("WARNING: Swerve Motor " + i + " is connected to port " + HardwareDevices.swerveMotors[i].getPortNumber() + ", should be port " + i, null);
            }
            if (HardwareDevices.swerveServos[i].getPortNumber() != i) {
                addRetainedTelemetry("WARNING: Swerve Servo " + i + " is connected to port " + HardwareDevices.swerveServos[i].getPortNumber() + ", should be port " + i, null);
            }
        }

        HardwareDevices.turretServoMaster = getHardwareDevice(ServoImplEx.class, "turretServoMaster");
        HardwareDevices.turretServoSlave = getHardwareDevice(ServoImplEx.class, "turretServoSlave");
        HardwareDevices.hoodServo = getHardwareDevice(ServoImplEx.class, "hoodServo");
        HardwareDevices.transferServoLow = getHardwareDevice(CRServoImplEx.class, "transferServoLow");
        HardwareDevices.transferServoHigh = getHardwareDevice(CRServoImplEx.class, "transferServoHigh");
        HardwareDevices.spindexServo = getHardwareDevice(CRServoImplEx.class, "spindexServo");
        HardwareDevices.intakeMotor = getHardwareDevice(DcMotorImplEx.class, "intakeMotor");
        HardwareDevices.flyWheelMotorMaster = getHardwareDevice(DcMotorImplEx.class, "flyWheelMotorMaster");
        HardwareDevices.flyWheelMotorSlave = getHardwareDevice(DcMotorImplEx.class, "flyWheelMotorSlave");

        //sensor
        HardwareDevices.colorSensor = getHardwareDevice(RevColorSensorV3.class, "colorSensor");
        HardwareDevices.beamBreak = getHardwareDevice(RevTouchSensor.class, "beamBreak");

        LinkedMotors flyWheelMotors = new LinkedMotors(HardwareDevices.flyWheelMotorMaster, HardwareDevices.flyWheelMotorSlave);
        HardwareDevices.flyWheelMotorSlave.setDirection(DcMotorImplEx.Direction.REVERSE);
        flyWheelMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LinkedServos turretServos = new LinkedServos(HardwareDevices.turretServoMaster, HardwareDevices.turretServoSlave);

        pathPlanner = new PathPlanner(telemetry, this);

        limelightLogic = new LimelightLogic(this, telemetry, HardwareDevices.limelight);
        huskyLensLogic1 = new HuskyLensLogic(this, RobotContainer.HardwareDevices.huskyLens1);
        huskyLensLogic2 = new HuskyLensLogic(this, RobotContainer.HardwareDevices.huskyLens2);
        turret = new Turret(this, flyWheelMotors, HardwareDevices.hoodServo, turretServos);
        HardwareDevices.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(this, HardwareDevices.intakeMotor);
        spindexer = new Spindexer(this, HardwareDevices.spindexServo, HardwareDevices.spindexAnalog, HardwareDevices.colorSensor);
        transfer = new Transfer(this, HardwareDevices.transferServoLow, HardwareDevices.transferServoHigh);

        indicatorLightFront = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightFront);
        indicatorLightBack = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightBack);
        allIndicatorLights = new IndicatorLighting.Group(this);
        allIndicatorLights.addLight(indicatorLightFront);
        allIndicatorLights.addLight(indicatorLightBack);

        drivetrain = new Drivetrain(this);
        headingPID = new HeadingPID(this);
        registerLoopTimer("teleOp");
        registerLoopTimer("drivetrainUpdater");
        registerLoopTimer("pinpointUpdater");
    }

    public void init() {
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        HardwareDevices.controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        HardwareDevices.expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_UPDATE_INTERVAL_MS);
    }

    public void start(OpMode opmode) {
        gamepadEx1 = new GamepadWrapper(opmode.gamepad1);
        gamepadEx2 = new GamepadWrapper(opmode.gamepad2);
        localizationUpdater = new LocalizationUpdater(this);
        localizationUpdater.start();
        drivetrainUpdater = new DrivetrainUpdater(this);
        drivetrainUpdater.start();
        telemetryLoopTimer.reset();
//        adgUpdater = new ADGUpdater(HardwareDevices.mux1, this);
//        adgUpdater.start();
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
        telemetry.addData("Control Hub Voltage", getVoltage(Constants.Robot.CONTROL_HUB_INDEX) + " V");
        telemetry.addData("Expansion Hub Voltage", getVoltage(Constants.Robot.EXPANSION_HUB_INDEX) + " V");
        telemetry.addData("Control Hub Current", getCurrent(Constants.Robot.CONTROL_HUB_INDEX) + " A");
        telemetry.addData("Expansion Hub Current", getCurrent(Constants.Robot.EXPANSION_HUB_INDEX) + " A");
        telemetry.addLine();
        telemetry.addData("Pinpoint X", Status.currentPose.getX(DistanceUnit.CM) + " cm");
        telemetry.addData("Pinpoint Y", Status.currentPose.getY(DistanceUnit.CM) + " cm");
        telemetry.addData("Pinpoint Heading", Status.currentHeading + "°");
        telemetry.addData("PINPOINT STATUS", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        telemetry.addLine();
        telemetry.addData("OpMode Avg Loop Time", (int) getRollingAverageLoopTime(opMode) + " ms");
        telemetry.addData("OpMode Loop Time", (int) getLoopTime(opMode) + " ms");
        telemetry.addLine();
        telemetry.addData("DriveTrain Avg Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        telemetry.addData("DriveTrain Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_MS + " ms");
        telemetry.addLine();
        telemetry.addData("Pinpoint Avg Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        telemetry.addData("Pinpoint Loop Time", (int) localizationUpdater.CURRENT_LOOP_TIME_MS + " ms");
        telemetry.addLine();
        telemetry.addData("Heading PID Target", headingPID.getTargetHeading());
        telemetry.addData("Heading PID Target Reached", Status.robotHeadingTargetReached);
        telemetry.addData("Heading PID Output", headingPID.calculate(Status.currentHeading));
        telemetry.addLine();
        telemetry.addData("Left Stick Y", gamepadEx1.leftStickY());
        telemetry.addData("Left Stick X", gamepadEx1.leftStickX());
        telemetry.addData("Right Stick Y", gamepadEx1.rightStickY());
        telemetry.addData("Right Stick X", gamepadEx1.rightStickX());
        telemetry.addLine();
        telemetry.addData("Motor 0 Current Velocity", swerveModules[0].motor.getVelocity());
        telemetry.addData("Motor 1 Current Velocity", swerveModules[1].motor.getVelocity());
        telemetry.addData("Motor 2 Current Velocity", swerveModules[2].motor.getVelocity());
        telemetry.addData("Motor 3 Current Velocity", swerveModules[3].motor.getVelocity());
        telemetry.addData("Field Oriented", Status.fieldOriented);
        telemetry.addData("Intake Enabled", Status.intakeEnabled);
//        telemetry.addData("Color Sensor", RobotContainer.HardwareDevices.colorSensor.getLightDetected());
//        telemetry.addData("Color Sensor Raw", RobotContainer.HardwareDevices.colorSensor.getRawLightDetected());
//        timer1.reset();
//        HardwareDevices.flyWheelMotorMaster.getVelocity();
//        telemetry.addData("timer1", timer1.milliseconds());
//        timer2.reset();
//        HardwareDevices.flyWheelMotorMaster.getVelocity();
//        telemetry.addData("timer2", timer2.milliseconds());
        double blue = HardwareDevices.colorSensor.blue();
        double green = HardwareDevices.colorSensor.green();

        if (blue > green) {
            telemetry.addLine("Purple");
        } else if (green > blue) {
            if (green < 100) {
                telemetry.addLine("Empty");
            } else {
                telemetry.addLine("Green");
            }
        }

        int selectedServo = -1;
        if (gamepadEx1.dpadUp.isPressed()) {
            selectedServo = 0;
        } else if (gamepadEx1.dpadRight.isPressed()) {
            selectedServo = 1;
        } else if (gamepadEx1.dpadDown.isPressed()) {
            selectedServo = 2;
        } else if (gamepadEx1.dpadLeft.isPressed()) {
            selectedServo = 3;
        }
        if (selectedServo >= 0) {
            telemetry.addLine();
            telemetry.addData("Selected Servo", selectedServo);
            telemetry.addData("Servo Angle", swerveModules[selectedServo].servo.getAngle());
            telemetry.addData("Servo Target", swerveServosPIDF[selectedServo].getTargetAngle());
            telemetry.addData("Servo Set Power", swerveServosPIDF[selectedServo].calculate());
            telemetry.addData("Servo Error", swerveServosPIDF[selectedServo].getError());
            telemetry.addData("Motor Target Power", swerveModules[selectedServo].motor.targetPower);
            telemetry.addData("Motor Current Velocity", swerveModules[selectedServo].motor.getVelocity());
            telemetry.addData("Motor Current Power", RobotContainer.HardwareDevices.swerveMotors[selectedServo].getPower());
        }
        telemetry.addLine();
        displayRetainedTelemetry();
        telemetry.update();
    }
}
