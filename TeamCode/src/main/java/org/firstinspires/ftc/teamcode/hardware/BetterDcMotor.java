package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterDcMotor {
    private DcMotorImplEx motor;

    private long lastTimePower;
    private long lastTimeVelocity;

    private double lastPower = 0;
    private double power = 0;
    private double lastVelocity = 0;
    private double velocity = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterDcMotor(DcMotorImplEx motor, int minTimeBetweenUpdates) {
        this.motor = motor;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public void updateSetPower(double power) {
        this.power = power;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimePower >= minTimeBetweenUpdates && power != lastPower) {
            motor.setPower(power);
            lastTimePower = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void updateSetVelocity(double velocity) {
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
