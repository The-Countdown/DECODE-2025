package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterServo {
    private ServoImplEx servo;

    private long lastTime;

    private double lastPosition = 0;
    private double position = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterServo(ServoImplEx servo, int minTimeBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public void updateSetPosition(double position) {
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
