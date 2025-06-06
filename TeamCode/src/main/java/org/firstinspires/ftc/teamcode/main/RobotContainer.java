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
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public final Map<String, LinkedList<Double>> loopTimesMap = new HashMap<>();
    public final Map<String, ElapsedTime> loopTimers = new HashMap<>();
    private final ArrayList<String> retainedTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> retainedTelemetryValues = new ArrayList<>();
    public static DcMotorImplEx[] driveMotors = new DcMotorImplEx[4];
    public static String[] driveMotorNames = new String[4];
    public DrivetrainUpdater drivetrainUpdater;
    public PinpointUpdater pinpointUpdater;
    public Pose2D pinpointPose;
    public DelayedActionManager delayedActionManager = new DelayedActionManager();
    public Drivetrain drivetrain;
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

        public static DcMotorImplEx[] driveMotors = new DcMotorImplEx[4];
        public static String[] driveMotorNames = new String[4];

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

        for (int i = 0; i < HardwareDevices.driveMotors.length; i++) {
            HardwareDevices.driveMotorNames[i] = "driveMotor" + (i);
            HardwareDevices.driveMotors[i] = getHardwareDevice(DcMotorImplEx.class, HardwareDevices.driveMotorNames[i]);
            HardwareDevices.driveMotors[i].setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        }
        HardwareDevices.driveMotors[1].setDirection(DcMotorImplEx.Direction.REVERSE);
        HardwareDevices.driveMotors[2].setDirection(DcMotorImplEx.Direction.REVERSE);

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
            telemetry.addLine("Turret has been loaded and flagged as functional!");
        } else {
            telemetry.addLine("Turret has not been loaded and flagged as non functional!");
        }
        turret = new Turret(this);
        turretFlywheel = new LinkedMotors(HardwareDevices.turretFlywheelMaster, HardwareDevices.turretFlywheelSlave);
        intake = new Intake(this);

        drivetrain = new Drivetrain(this);
        registerLoopTimer("teleOp");
        registerLoopTimer("drivetrainUpdater");
        registerLoopTimer("pinpointUpdater");
    }

    public void init() {
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        HardwareDevices.controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        HardwareDevices.expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2"); // this should be 1 because the expansion hub on the swerve is that one, we should upgrade to 2 though
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
            telemetry.addLine("Failed to load hardware class: " + hardwareClass.toString()); // HARDWARE CLASS IS NULL, THIS WILL ERROR! fix it however you want cole
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

    public void telemetry (int currentServo, double offset, double CURRENT_LOOP_TIME_MS, double CURRENT_LOOP_TIME_AVG_MS, Gamepad gamepad) {
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
        opMode.telemetry.addData("Left Stick Y", gamepad.left_stick_y);
        opMode.telemetry.addLine();
        displayRetainedTelemetry();
        opMode.telemetry.update();
    }
}
