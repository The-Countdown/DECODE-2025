package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class BetterDcMotor {
    private static DcMotorImplEx motor;

    private long lastTimePower;
    private long lastTimeVelocity;

    private double lastPower = 0;
    private double power = 0;
    private double lastVelocity = 0;
    private double velocity = 0;

    public BetterDcMotor(DcMotorImplEx motor) {
        this.motor = motor;
    }

    public void updateSetPower(double power, int minTimeBetweenUpdates) {
        this.power = power;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimePower >= minTimeBetweenUpdates && power != lastPower) {
            motor.setPower(power);
            lastTimePower = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void updateSetVelocity(double velocity, int minTimeBetweenUpdates) {
        this.velocity = velocity;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeVelocity >= minTimeBetweenUpdates && velocity != lastVelocity) {
            motor.setVelocity(velocity);
            lastTimeVelocity = System.currentTimeMillis();
            this.lastVelocity = velocity;
        }
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior mode) {
        motor.setZeroPowerBehavior(mode);
    }

    public void setMode(DcMotorImplEx.RunMode mode) {
        motor.setMode(mode);
    }

    public void setDirection(DcMotorImplEx.Direction direction) {
        motor.setDirection(direction);
    }
    
    public double getPower() {
        return this.power;
    }

    public double getVelocity() {
        return this.velocity;
    }
}
