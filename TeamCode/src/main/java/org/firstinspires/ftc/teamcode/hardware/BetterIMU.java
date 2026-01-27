package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU.CalibrationData;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

// enum SensorMode {
//     IMU
// }

public class BetterIMU {
    // private SensorMode currentMode;
    private AdafruitBNO055IMU imu;
    private double angleOffset = 0;
    private BNO055IMU.CalibrationData calibration;

    public BetterIMU(AdafruitBNO055IMU imu) {
        this.imu = imu;
        this.angleOffset = 0;

        this.calibration = new BNO055IMU.CalibrationData();

        // super(deviceClient, true); // IDK to use true or false
        // setMode(SensorMode.IMU);
    }

    // Return the yaw in degrees
    public double getAngle() {
        return imu.getAngularOrientation().firstAngle - angleOffset;
    }

    public void resetAngle() {
        this.angleOffset = imu.getAngularOrientation().firstAngle;
    }

    public void setAngleOffset(double offsetAngle) {
        this.angleOffset = imu.getAngularOrientation().firstAngle - offsetAngle;
    }

    public void initialize(BNO055IMU.Parameters params) {
        imu.initialize(params);
    }

    public BNO055IMU.CalibrationData readCalibrationData() {
        return imu.readCalibrationData();
    }

    // public synchronized void setMode(SensorMode mode) {
    //     this.currentMode = mode;

    //     switch (mode) {
    //         case IMU:
    //             write8(BNO055IMU.Register.OPR_MODE, 0x07);
    //     }
    // }
}
