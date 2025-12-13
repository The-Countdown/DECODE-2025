package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterColorSensor {
    private RevColorSensorV3 sensor;

    private long lastTimeRed;
    private long lastTimeBlue;
    private long lastTimeGreen;

    private double red = 0;
    private double blue = 0;
    private double green = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterColorSensor(RevColorSensorV3 sensor, int minTimeBetweenUpdates) {
        this.sensor = sensor;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public double updateRed() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeRed >= minTimeBetweenUpdates) {
            red = sensor.red();
            lastTimeRed = System.currentTimeMillis();
        }
        return red;
    }

    public double updateBlue() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeBlue >= minTimeBetweenUpdates) {
            blue = sensor.blue();
            lastTimeBlue = System.currentTimeMillis();
        }
        return blue;
    }

    public double updateGreen() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeGreen >= minTimeBetweenUpdates) {
            green = sensor.green();
            lastTimeGreen = System.currentTimeMillis();
        }
        return green;
    }

    public double getDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }
}
