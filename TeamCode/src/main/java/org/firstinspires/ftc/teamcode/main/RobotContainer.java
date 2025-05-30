package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.HeadingPID;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.drivetrain.SwervePIDF;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.IndicatorLighting;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;

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
    public boolean turretFunctional = false;
    public final Map<String, LinkedList<Double>> loopTimesMap = new HashMap<>();
    public final Map<String, ElapsedTime> loopTimers = new HashMap<>();
    private final ArrayList<String> retainedTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> retainedTelemetryValues = new ArrayList<>();
    public final SwerveModule[] swerveModules = new SwerveModule[Constants.NUM_SWERVE_MOTORS];
    public SwervePIDF[] swerveServosPIDF = new SwervePIDF[Constants.NUM_SWERVE_SERVOS];
    public DrivetrainUpdater drivetrainUpdater;
    public PinpointUpdater pinpointUpdater;
    public Pose2D pinpointPose;
    public DelayedActionManager delayedActionManager = new DelayedActionManager();
    public Drivetrain drivetrain;
    public HeadingPID headingPID;
    public IndicatorLighting.Light indicatorLightFrontLeft;
    public IndicatorLighting.Light indicatorLightFrontRight;
    public IndicatorLighting.Light indicatorLightBack;
    public IndicatorLighting.Group allIndicatorLights = new IndicatorLighting.Group();
    public Turret turret = null;
    public LinkedMotors turretFlywheel = null;
    public Intake intake = null;

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static LynxModule controlHub;
        public static LynxModule expansionHub;
        public static IMU imu;

        public static GoBildaPinpoint pinpoint;
        public static Limelight3A limelight;
        public static RevColorSensorV3 flashlight;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLightFrontLeft;
        public static ServoImplEx indicatorLightFrontRight;
        public static ServoImplEx indicatorLightBack;

        // Gobilda 5000 Series
        public static DcMotorImplEx[] swerveMotors = new DcMotorImplEx[Constants.NUM_SWERVE_MOTORS];
            public static String[] motorNames = new String[Constants.NUM_SWERVE_MOTORS];

        // Axon Mini+
        public static CRServoImplEx[] swerveServos = new CRServoImplEx[Constants.NUM_SWERVE_SERVOS];
            public static String[] servoNames = new String[Constants.NUM_SWERVE_SERVOS];

        public static AnalogInput[] swerveAnalogs = new AnalogInput[Constants.NUM_SWERVE_ANALOGS];
            public static String[] analogNames = new String[Constants.NUM_SWERVE_ANALOGS];

        // Turret
        public static DcMotorImplEx turretFlywheelMaster;
        public static DcMotorImplEx turretFlywheelSlave;
        public static DcMotorImplEx turretRotation;
        public static DcMotorImplEx turretIntakeMotor;
        public static DcMotorImplEx intakeMotor;
        public static CRServoImplEx turretIntakeServo;
        public static CRServoImplEx lateralConveyorServo;
        public static CRServoImplEx longitudinalConveyorServo;
        public static ServoImplEx turretArcServo;
        public static AnalogInput turretEncoder;
    }

    public RobotContainer(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        HardwareDevices.imu = getHardwareDevice(IMU.class, "imu");
        HardwareDevices.imu.initialize(Constants.imuParameters);

        HardwareDevices.pinpoint = getHardwareDevice(GoBildaPinpoint.class, "pinpoint");
        HardwareDevices.pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET_MM, Constants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        HardwareDevices.pinpoint.setEncoderResolution(Constants.PINPOINT_ODOM_POD);
        HardwareDevices.pinpoint.setEncoderDirections(Constants.PINPOINT_X_ENCODER_DIRECTION, Constants.PINPOINT_Y_ENCODER_DIRECTION);

        HardwareDevices.limelight = getHardwareDevice(Limelight3A.class, "limelight");
        HardwareDevices.flashlight = getHardwareDevice(RevColorSensorV3.class, "flashlight");

        HardwareDevices.indicatorLightFrontLeft = getHardwareDevice(ServoImplEx.class, "indicatorLightFrontLeft");
        HardwareDevices.indicatorLightFrontRight = getHardwareDevice(ServoImplEx.class, "indicatorLightFrontRight");
        HardwareDevices.indicatorLightBack = getHardwareDevice(ServoImplEx.class, "indicatorLightBack");

        if (Constants.TURRET_BOT_ACTIVE) {
            HardwareDevices.turretFlywheelMaster = getHardwareDevice(DcMotorImplEx.class, "turretFlywheelMaster");
            HardwareDevices.turretFlywheelSlave = getHardwareDevice(DcMotorImplEx.class, "turretFlywheelSlave");
            HardwareDevices.turretFlywheelMaster.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
            HardwareDevices.turretFlywheelSlave.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
            HardwareDevices.turretRotation = getHardwareDevice(DcMotorImplEx.class, "turretRotation");
            HardwareDevices.turretRotation.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
            HardwareDevices.turretIntakeMotor = getHardwareDevice(DcMotorImplEx.class, "turretIntakeMotor");
            HardwareDevices.intakeMotor = getHardwareDevice(DcMotorImplEx.class, "intakeMotor");
            HardwareDevices.turretArcServo = getHardwareDevice(ServoImplEx.class, "turretArcServo");
            HardwareDevices.turretIntakeServo = getHardwareDevice(CRServoImplEx.class, "turretIntakeServo");
            HardwareDevices.lateralConveyorServo = getHardwareDevice(CRServoImplEx.class, "lateralConveyorServo");
            HardwareDevices.longitudinalConveyorServo = getHardwareDevice(CRServoImplEx.class, "longitudinalConveyorServo");
            HardwareDevices.turretEncoder = getHardwareDevice(AnalogInput.class, "turretEncoder");
            if (HardwareDevices.turretFlywheelMaster != null && HardwareDevices.turretRotation != null) { // Assume turret wants to be loaded
                turretFunctional = true;
                telemetry.addLine("Turret has been loaded and flagged as functional!");
            } else {
                telemetry.addLine("Turret has not been loaded and flagged as non functional!");
                turretFunctional = false;
            }
            turret = new Turret(this);
            turretFlywheel = new LinkedMotors(HardwareDevices.turretFlywheelMaster, HardwareDevices.turretFlywheelSlave);
            intake = new Intake(this);
        }

        if (!turretFunctional) {
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

                swerveServosPIDF[i] = new SwervePIDF(this, i, HardwareDevices.swerveServos[i]);
                swerveModules[i] = new SwerveModule(this, HardwareDevices.swerveMotors[i], HardwareDevices.swerveServos[i], swerveServosPIDF[i], HardwareDevices.swerveAnalogs[i], Constants.SWERVE_POWER_MULTIPLIER[i], i);

                if (Constants.SERVO_ANALOG_ACTIVE) {
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
        }

        indicatorLightFrontLeft = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightFrontLeft);
        indicatorLightFrontRight = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightFrontRight);
        indicatorLightBack = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightBack);
        allIndicatorLights.addLight(indicatorLightFrontLeft);
        allIndicatorLights.addLight(indicatorLightFrontRight);
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
    }

    public void start(OpMode opmode) {
        gamepadEx1 = new GamepadWrapper(opmode.gamepad1);
        gamepadEx2 = new GamepadWrapper(opmode.gamepad2);
        pinpointUpdater = new PinpointUpdater(this);
        pinpointUpdater.start();
        drivetrainUpdater = new DrivetrainUpdater(this);
        drivetrainUpdater.start();
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

    // This is for hardware error that are critical and code execution should stop to tell the user of the error.
    public void testCriticalHardwareDevice(Object hardwareClass) {
        if (hardwareClass == null) {
            telemetry.log().clear();
            telemetry.addLine("Failed to load hardware class: " + hardwareClass.toString());
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
        if (times.size() > Constants.LOOP_AVERAGE_WINDOW_SIZE) {
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

    public void telemetry (int currentServo, double CURRENT_LOOP_TIME_MS, double CURRENT_LOOP_TIME_AVG_MS) {
        opMode.telemetry.addData("Control Hub Voltage", getVoltage(Constants.CONTROL_HUB_INDEX) + " V");
        opMode.telemetry.addData("Expansion Hub Voltage", getVoltage(Constants.EXPANSION_HUB_INDEX) + " V");
        opMode.telemetry.addData("Control Hub Current", getCurrent(Constants.CONTROL_HUB_INDEX) + " A");
        opMode.telemetry.addData("Expansion Hub Current", getCurrent(Constants.EXPANSION_HUB_INDEX) + " A");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Pinpoint X", PinpointUpdater.currentPose.getX(DistanceUnit.CM) + " cm");
        opMode.telemetry.addData("Pinpoint Y", PinpointUpdater.currentPose.getY(DistanceUnit.CM) + " cm");
        opMode.telemetry.addData("Pinpoint Heading", PinpointUpdater.currentHeading + "°");
        opMode.telemetry.addData("PINPOINT STATUS", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        opMode.telemetry.addLine();
        opMode.telemetry.addData("TeleOp Avg Loop Time", (int) CURRENT_LOOP_TIME_AVG_MS + " ms");
        opMode.telemetry.addData("TeleOp Loop Time", (int) CURRENT_LOOP_TIME_MS + " ms");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("DriveTrain Avg Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        opMode.telemetry.addData("DriveTrain Loop Time", (int) drivetrainUpdater.CURRENT_LOOP_TIME_MS + " ms");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Pinpoint Avg Loop Time", (int) pinpointUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        opMode.telemetry.addData("Pinpoint Loop Time", (int) pinpointUpdater.CURRENT_LOOP_TIME_MS + " ms");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Heading PID Target", headingPID.getTargetHeading());
        opMode.telemetry.addData("Heading PID Target Reached", Status.robotHeadingTargetReached);
        opMode.telemetry.addData("Heading PID Output", headingPID.calculate(PinpointUpdater.currentHeading));
        if (currentServo >= 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Selected Servo", currentServo);
            opMode.telemetry.addData("Servo Angle", swerveModules[currentServo].servo.getAngle());
            opMode.telemetry.addData("Servo Target", swerveServosPIDF[currentServo].getTargetAngle());
            opMode.telemetry.addData("Servo Set Power", swerveServosPIDF[currentServo].calculate());
            opMode.telemetry.addData("Servo Error", swerveServosPIDF[currentServo].getError());
            opMode.telemetry.addData("Motor Target Power", swerveModules[currentServo].motor.targetPower);
            opMode.telemetry.addData("Motor Current Power", RobotContainer.HardwareDevices.swerveMotors[currentServo].getPower());
        }
        opMode.telemetry.addLine();
        displayRetainedTelemetry();
        opMode.telemetry.update();
    }
}
