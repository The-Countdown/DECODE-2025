package org.firstinspires.ftc.teamcode.main;

import android.os.Handler;
import android.os.Looper;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.HeadingPID;
import org.firstinspires.ftc.teamcode.drivetrain.SwerveModule;
import org.firstinspires.ftc.teamcode.drivetrain.SwervePIDF;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;

import java.util.HashMap;
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
    private final Map<String, Telemetry.Item> retainedItems = new HashMap<>();
    private final Handler handler = new Handler(Looper.getMainLooper());
    public final SwerveModule[] swerveModules = new SwerveModule[Constants.NUM_SWERVE_MOTORS];
    public SwervePIDF[] swerveServosPIDF = new SwervePIDF[Constants.NUM_SWERVE_SERVOS];
    public DrivetrainUpdater drivetrainUpdater = new DrivetrainUpdater(this);

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static IMU imu;

        public static GoBildaPinpoint pinpoint;
        public static Limelight3A limelight;
        public static RevColorSensorV3 flashlight;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLight;

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
        this.telemetry = opMode.telemetry;

        telemetry.setAutoClear(false);

        HardwareDevices.imu = getHardwareDevice(IMU.class, "imu");
        HardwareDevices.imu.initialize(Constants.imuParameters);

        HardwareDevices.pinpoint = getHardwareDevice(GoBildaPinpoint.class, "pinpoint");
                HardwareDevices.pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET_MM, Constants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
                HardwareDevices.pinpoint.setEncoderResolution(Constants.PINPOINT_ODOM_POD);
                HardwareDevices.pinpoint.setEncoderDirections(Constants.PINPOINT_X_ENCODER_DIRECTION, Constants.PINPOINT_Y_ENCODER_DIRECTION);

        HardwareDevices.limelight = getHardwareDevice(Limelight3A.class, "limelight");
        HardwareDevices.flashlight = getHardwareDevice(RevColorSensorV3.class, "flashlight");

        HardwareDevices.indicatorLight = getHardwareDevice(ServoImplEx.class, "indicatorLight");

        for (int i = 0; i < HardwareDevices.swerveMotors.length; i++) {
            HardwareDevices.motorNames[i] = "swerveMotor" + (i);
            HardwareDevices.swerveMotors[i] = getHardwareDevice(DcMotorImplEx.class, HardwareDevices.motorNames[i]);
            if (i == 0 || i == 2) { // TODO: Find which motors to reverse
                HardwareDevices.swerveMotors[i].setDirection(DcMotorImplEx.Direction.REVERSE);
            }
            HardwareDevices.swerveMotors[i].setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        }

        for (int i = 0; i < HardwareDevices.swerveServos.length; i++) {
            HardwareDevices.servoNames[i] = "swerveServo" + (i);
            HardwareDevices.swerveServos[i] = getHardwareDevice(CRServoImplEx.class, HardwareDevices.servoNames[i]);
        }

        for (int i = 0; i < HardwareDevices.swerveAnalogs.length; i++) {
            HardwareDevices.analogNames[i] = "swerveAnalog" + (i);
            HardwareDevices.swerveAnalogs[i] = getHardwareDevice(AnalogInput.class, HardwareDevices.analogNames[i]);
        }

        HardwareDevices.turretFlywheelMaster = getHardwareDevice(DcMotorImplEx.class, "turretFlywheelMaster");
        HardwareDevices.turretFlywheelSlave = getHardwareDevice(DcMotorImplEx.class, "turretFlywheelSlave");
        HardwareDevices.turretRotation = getHardwareDevice(DcMotorImplEx.class, "turretRotation");
        HardwareDevices.turretIntakeMotor = getHardwareDevice(DcMotorImplEx.class, "turretIntakeMotor");
        HardwareDevices.turretArcServo = getHardwareDevice(ServoImplEx.class, "turretArcServo");
        HardwareDevices.turretIntakeServo = getHardwareDevice(CRServoImplEx.class, "turretIntakeServo");
        HardwareDevices.lateralConveyorServo = getHardwareDevice(CRServoImplEx.class, "lateralConveyorServo");
        HardwareDevices.longitudinalConveyorServo = getHardwareDevice(CRServoImplEx.class, "longitudinalConveyorServo");
        HardwareDevices.turretEncoder = getHardwareDevice(AnalogInput.class, "turretEncoder");

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i] = new SwerveModule(this,
                    HardwareDevices.swerveMotors[i],
                    HardwareDevices.swerveServos[i],
                    HardwareDevices.swerveAnalogs[i],
                    i);
        }

        for (int i = 0; i < swerveServosPIDF.length; i++) {
            swerveServosPIDF[i] = new SwervePIDF(this, i, HardwareDevices.swerveServos[i]);
        }

        for (int i = 0; i < HardwareDevices.swerveMotors.length; i++) {
            int portNumber = HardwareDevices.swerveMotors[i].getPortNumber();
            if (portNumber != i) {
                addRetained("WARNING: Swerve Motor " + i + " is connected to port " + portNumber + ", should be port " + i, null);
            }
        }
        for (int i = 0; i < HardwareDevices.swerveServos.length; i++) {
            int portNumber = HardwareDevices.swerveServos[i].getPortNumber();
            if (portNumber != i) {
                addRetained("WARNING: Swerve Servo " + i + " is connected to port " + portNumber + ", should be port " + i, null);
            }
        }
        for (int i = 0; i < HardwareDevices.swerveAnalogs.length; i++) {
            // Get the port number (last character) from the connection info string.
            int portNumber = Character.getNumericValue(HardwareDevices.swerveAnalogs[i].getConnectionInfo().charAt(HardwareDevices.swerveAnalogs[i].getConnectionInfo().length() - 1));
            if (portNumber != i) {
                addRetained("WARNING: Swerve Analog Encoder " + i + " is connected to port " + portNumber + ", should be port " + i, null);
            }
        }

        drivetrainUpdater.start();
    }

    public void init() {
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Retrieves a hardware device from the hardware map.
     *
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
            telemetry.addLine("Error: " + e.getMessage());
            return null; // or throw the exception if you prefer
        }
    }

    /**
     * Add or update a retained line.
     * Calling this with the same caption again will just change its value.
     */
    public void addRetained(String caption, Object value) {
        Telemetry.Item item = retainedItems.get(caption);
        if (item == null) {
            if (value == null) {
                item = (Telemetry.Item) telemetry.addLine(caption);
            } else {
                item = telemetry.addData(caption, value);
            }
            item.setRetained(true);
            retainedItems.put(caption, item);
        } else {
            if (value != null) {
                item.setValue(value);
            }
        }
    }

    /**
     * Runs a sequence of actions with specified delays between them.
     * This method takes a list of actions (Runnable objects) and a corresponding list of delays.
     * It ensures that each action is executed after the specified delay from the previous action.
     *
     * @param actions A list of Runnable actions to be executed in sequence.
     * @param delays  A list of delay times (in milliseconds) corresponding to each action.
     *                The size of this list must be the same as the 'actions' list.
     * @throws IllegalArgumentException If the 'actions' and 'delays' lists have different lengths.
     */
    public void runActionSequence(List<Runnable> actions, List<Long> delays) {
        if (actions.size() != delays.size()) {
            throw new IllegalArgumentException("Actions and delays must have the same length.");
        }
        // Start the recursive process of running actions with delays, starting at index 0.
        runActionSequenceInternal(actions, delays, 0);
    }
    /**
     * Internal recursive method to run the action sequence.
     * This method recursively executes the actions in the 'actions' list with the delays specified
     * in the 'delays' list. It uses a Handler to schedule each action to run after the specified delay.
     *
     * @param actions A list of Runnable actions to be executed.
     * @param delays  A list of delay times (in milliseconds) for each action.
     * @param index   The current index of the action and delay to be processed.
     */
    private void runActionSequenceInternal(List<Runnable> actions, List<Long> delays, int index) {
        // Base case: If the index is past the end of the list, we're done.
        if (index >= actions.size()) return;

        // Run the action at the current index.
        actions.get(index).run();

        // Schedule the next action to run after the specified delay.
        // Recursively call this method for the next action (index + 1).
        handler.postDelayed(() -> runActionSequenceInternal(actions, delays, index + 1), delays.get(index));
    }

    /**
     * Refreshes the data from all Lynx Modules (hubs) by clearing their bulk data cache.
     *
     * This method is crucial for ensuring that the robot is operating with the most up-to-date
     * sensor and motor data. The Lynx Modules use a bulk data cache to optimize data transfer.
     * However, if data in this cache becomes stale, the robot's actions might be based on
     * outdated information.
     *
     * This method should be called periodically or whenever you suspect that the data in the
     * bulk cache might be outdated. Common scenarios include:
     * - At the start of a new control loop iteration in teleop or autonomous.
     * - After a significant delay or pause in the robot's operation.
     * - Before reading critical sensor values that need to be absolutely current.
     * - If there is a change in the bulk caching mode.
     *
     * Calling this method ensures that the next time you read data from the hubs, the latest
     * information will be fetched, rather than possibly outdated cached data.
     */
    public void refreshData() {
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getVoltage() {
        // Sets the voltage to -1 in case it is not set by the LynxModule, makes for easy debugging
        double voltage = -1;
        for (LynxModule hub : RobotContainer.HardwareDevices.allHubs) {
            voltage = hub.getInputVoltage(VoltageUnit.VOLTS);
        }

        return voltage;
    }

    public double getCurrent() {
        // Sets the current to -1 in case it is not set by the LynxModule, makes for easy debugging
        double current = -1;
        for (LynxModule hub : RobotContainer.HardwareDevices.allHubs) {
            current = hub.getCurrent(CurrentUnit.MILLIAMPS);
        }

        return current;
    }

    public Drivetrain drivetrain = new Drivetrain(this);
    public HeadingPID headingPID = new HeadingPID(this);
    public IndicatorLight indicatorLight = new IndicatorLight(this);
    public Turret turret = new Turret(this);
    public LinkedMotors turretFlywheel = new LinkedMotors(HardwareDevices.turretFlywheelMaster, HardwareDevices.turretFlywheelSlave);
}
