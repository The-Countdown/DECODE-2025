package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterCRServo {
    private CRServoImplEx servo;

    private long lastTime;

    private double lastPower = 0;
    private double power = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterCRServo(CRServoImplEx servo, int minTimeBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public void updateSetPower(double power) {
        this.power = power;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates && power != lastPower) {
            servo.setPower(power);
            lastTime = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void setDirection(CRServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPower() {
        return this.power;
    }
}
