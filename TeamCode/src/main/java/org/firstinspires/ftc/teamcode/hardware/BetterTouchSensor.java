package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevTouchSensor;

import java.util.Timer;

public class BetterTouchSensor {
    private static RevTouchSensor sensor;

    private long lastTime;

    public boolean pressed = false;

    public BetterTouchSensor(RevTouchSensor sensor) {
        this.sensor = sensor;
    }

    public void update(int minTimeBetweenUpdates) {
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
