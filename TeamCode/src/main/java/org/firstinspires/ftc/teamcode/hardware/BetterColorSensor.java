package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class BetterColorSensor {
    private static RevColorSensorV3 sensor;

    private long lastTimeRed;
    private long lastTimeBlue;
    private long lastTimeGreen;

    private double red = 0;
    private double blue = 0;
    private double green = 0;

    public BetterColorSensor(RevColorSensorV3 sensor) {
        this.sensor = sensor;
    }

    public double updateRed(int minTimeBetweenUpdates) {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeRed >= minTimeBetweenUpdates) {
            red = sensor.red();
            lastTimeRed = System.currentTimeMillis();
        }
        return red;
    }

    public double updateBlue(int minTimeBetweenUpdates) {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeBlue >= minTimeBetweenUpdates) {
            blue = sensor.blue();
            lastTimeBlue = System.currentTimeMillis();
        }
        return blue;
    }

    public double updateGreen(int minTimeBetweenUpdates) {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeGreen >= minTimeBetweenUpdates) {
            green = sensor.green();
            lastTimeGreen = System.currentTimeMillis();
        }
        return green;
    }
}
