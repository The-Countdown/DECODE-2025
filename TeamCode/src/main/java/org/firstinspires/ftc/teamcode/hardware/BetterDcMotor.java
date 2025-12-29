package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterDcMotor extends Thread {
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

    @Override
    public void run() {
        while (true) {
            updateSetPower(power);
            updateSetVelocity(velocity);

            try {
                Thread.sleep(10); // Update this
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
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

    public void setPower(double power) {
        this.power = power;
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

    public void setVelocity(double velocity) {
        this.velocity = velocity;
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

    public void setPIDF(PIDFCoefficients pidf) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    
    public double getPower() {
        return this.power;
    }

//    public double getVelocity() {
//        return this.velocity;
//    }
    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return motor.getCurrent(currentUnit);
    }
}
