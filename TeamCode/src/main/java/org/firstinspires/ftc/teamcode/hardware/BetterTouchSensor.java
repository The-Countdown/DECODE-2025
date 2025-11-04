package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevTouchSensor;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterTouchSensor {
    private RevTouchSensor sensor;

    private long lastTime;

    public boolean pressed = false;

    private double minTimeBetweenUpdates = 0;

    public BetterTouchSensor(RevTouchSensor sensor, int minTimeBetweenUpdates) {
        this.sensor = sensor;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public void update() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates) {
            pressed = sensor.isPressed();
            lastTime = System.currentTimeMillis();
        }
    }

    public boolean isPressed() {
        return pressed;
    }
}
