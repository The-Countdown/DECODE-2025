package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterThreadedDcMotor extends Thread {
    private boolean enabled = false;
    private DcMotorImplEx motor;

    private long lastTimePower;
    private long lastTimeVelocity;

    private double lastPower = 0;
    private double power = 0;
    private double lastVelocity = 0;
    private double velocity = 0;

    private int minTimeBetweenUpdates = 0;

    public BetterThreadedDcMotor(DcMotorImplEx motor, int minTimeBetweenUpdates) {
        this.motor = motor;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    @Override
    public void run() {
        while (enabled) {
            updateSetPower(power);
            updateSetVelocity(velocity);

            try {
                Thread.sleep(minTimeBetweenUpdates);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public synchronized void updateSetPower(double power) {
        this.power = power;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimePower >= minTimeBetweenUpdates && power != lastPower) {
            motor.setPower(power);
            lastTimePower = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void setPower(double power) {
        this.power = power;
    }

    public synchronized void updateSetVelocity(double velocity) {
        this.velocity = velocity;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeVelocity >= minTimeBetweenUpdates && velocity != lastVelocity) {
            motor.setVelocity(velocity);
            lastTimeVelocity = System.currentTimeMillis();
            this.lastVelocity = velocity;
        }
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public synchronized void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior mode) {
        motor.setZeroPowerBehavior(mode);
    }

    public synchronized void setMode(DcMotorImplEx.RunMode mode) {
        motor.setMode(mode);
    }

    public synchronized void setDirection(DcMotorImplEx.Direction direction) {
        motor.setDirection(direction);
    }
    
    public double getPower() {
        return this.power;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public void enable() {
        enabled = true;   
    }

    public void disable() {
        enabled = false;
    }
}
