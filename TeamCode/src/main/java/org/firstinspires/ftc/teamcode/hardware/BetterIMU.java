package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

// enum SensorMode {
//     IMU
// }

public class BetterIMU extends AdafruitBNO055IMU {
    // private SensorMode currentMode;
    private BNO055IMU.Parameters params;

    public BetterIMU(I2cDeviceSynch deviceClient) {
        super(deviceClient, true); // IDK to use true or false
        // setMode(SensorMode.IMU);
    }

    // public synchronized void setMode(SensorMode mode) {
    //     this.currentMode = mode;

    //     switch (mode) {
    //         case IMU:
    //             write8(BNO055IMU.Register.OPR_MODE, 0x07);
    //     }
    // }
}
