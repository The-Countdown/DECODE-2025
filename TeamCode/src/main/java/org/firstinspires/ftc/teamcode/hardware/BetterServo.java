package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BetterServo {
    private static ServoImplEx servo;

    private long lastTime;

    private double lastPosition = 0;
    private double position = 0;

    public BetterServo(ServoImplEx servo) {
        this.servo = servo;
    }

    public void updateSetPosition(double position, int minTimeBetweenUpdates) {
        this.position = position;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates && position != lastPosition) {
            servo.setPosition(position);
            lastTime = System.currentTimeMillis();
            this.lastPosition = position;
        }
    }

    public void setDirection(ServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPosition() {
        return this.position;
    }
}
