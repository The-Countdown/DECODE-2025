package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

public class BetterCRServo {
    private static CRServoImplEx servo;

    private long lastTime;

    private double lastPower = 0;
    private double power = 0;

    public BetterCRServo(CRServoImplEx servo) {
        this.servo = servo;
    }

    public void updateSetPower(double power, int minTimeBetweenUpdates) {
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
