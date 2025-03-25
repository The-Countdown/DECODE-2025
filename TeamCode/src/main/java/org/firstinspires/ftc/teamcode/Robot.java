package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Robot {
    HardwareMap hardwareMap;
    LinearOpMode opMode;

    public static class HardwareDevices {
        public static IMU imu;
        public static VoltageSensor voltageSensor;
        public static Limelight3A limelight;
        public static RevColorSensorV3 flashLight;

        //Like a coordinate plane https://www.mathplanet.com/education/algebra-1/visualizing-linear-functions/the-coordinate-plane,
        // the first quadrant is 0, the second is 1, etc. relating to the positions of the modules on the robot
        public static DcMotorEx[] swerveMotors = new DcMotorEx[4];
            public static String[] motorNames = new String[4];

        public static CRServoImplEx[] swerveServos = new CRServoImplEx[4];
            public static String[] servoNames = new String[4];

        public static AnalogInput[] swerveAnalogs = new AnalogInput[4];
            public static String[] analogNames = new String[4];
    }

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        HardwareDevices.imu = hardwareMap.get(IMU.class, HardwareDevices.imu.getDeviceName());
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        HardwareDevices.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

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
    }
    public SwerveBase swerveBase = new SwerveBase(this);
}
