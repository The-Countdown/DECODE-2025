package org.firstinspires.ftc.teamcode;

import android.os.Looper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;
import android.os.Handler;

public class Robot {
    HardwareMap hardwareMap;
    OpMode opMode;
    public boolean running = false;
    private static final Handler handler = new Handler(Looper.getMainLooper());
    public SwerveModule[] swerveModules = new SwerveModule[4];
    public SwerveServoPIDF[] swerveServosPIDF = new SwerveServoPIDF[HardwareDevices.swerveServos.length];
    private final ThreadedPIDF threadedPIDF;

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;

        public static IMU imu;
        public static Limelight3A limelight;
        public static RevColorSensorV3 flashLight;

        /*Like a coordinate plane https://www.mathplanet.com/education/algebra-1/visualizing-linear-functions/the-coordinate-plane,
         the first quadrant is 0, the second is 1, etc. relating to the positions of the modules on the robot */
        public static DcMotorEx[] swerveMotors = new DcMotorEx[4];
            public static String[] motorNames = new String[4];

        public static CRServoImplEx[] swerveServos = new CRServoImplEx[4];
            public static String[] servoNames = new String[4];

        public static AnalogInput[] swerveAnalogs = new AnalogInput[4];
            public static String[] analogNames = new String[4];
    }

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        HardwareDevices.imu = hardwareMap.get(IMU.class, HardwareDevices.imu.getDeviceName());
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        HardwareDevices.limelight = hardwareMap.get(Limelight3A.class, "limeLight");
            HardwareDevices.flashLight = hardwareMap.get(RevColorSensorV3.class, "flashLight");

        for (int i = 0; i < HardwareDevices.swerveMotors.length; i++) {
            HardwareDevices.motorNames[i] = "swerveMotor" + (i);
            HardwareDevices.swerveMotors[i] = hardwareMap.get(DcMotorEx.class, HardwareDevices.motorNames[i]);
        }

        for (int i = 0; i < HardwareDevices.swerveServos.length; i++) {
            HardwareDevices.servoNames[i] = "swerveServo" + (i);
            HardwareDevices.swerveServos[i] = hardwareMap.get(CRServoImplEx.class, HardwareDevices.servoNames[i]);
        }

        for (int i = 0; i < HardwareDevices.swerveAnalogs.length; i++) {
            HardwareDevices.analogNames[i] = "swerveAnalog" + (i);
            HardwareDevices.swerveAnalogs[i] = hardwareMap.get(AnalogInput.class, HardwareDevices.analogNames[i]);
        }

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i] = new SwerveModule(this,
                    HardwareDevices.swerveMotors[i],
                    HardwareDevices.swerveServos[i],
                    HardwareDevices.swerveAnalogs[i],
                    i);
        }

        for (int i = 0; i < swerveServosPIDF.length; i++) {
            swerveServosPIDF[i] = new SwerveServoPIDF(this, i, HardwareDevices.swerveServos[i]);
        }

        threadedPIDF = new ThreadedPIDF(this);
        threadedPIDF.start();
    }

    public void runActionSequence(List<Runnable> actions, List<Long> delays, int index) {
        if (actions.size() != delays.size()) {
            throw new IllegalArgumentException("Actions and delays must have the same length.");
        }
        if (index >= actions.size()) return;

        actions.get(index).run();

        handler.postDelayed(() -> runActionSequence(actions, delays, index + 1), delays.get(index));
    }

    public void refreshData() {
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getVoltage() {
        double voltage = -1;
        for (LynxModule hub : Robot.HardwareDevices.allHubs) {
            refreshData();
            voltage = hub.getInputVoltage(VoltageUnit.VOLTS);
        }

        return voltage;
    }

    public double getCurrent() {
        double current = -1;
        for (LynxModule hub : Robot.HardwareDevices.allHubs) {
            refreshData();
            current = hub.getCurrent(CurrentUnit.MILLIAMPS);
        }

        return current;
    }
}
