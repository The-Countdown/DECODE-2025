package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterThreadedCRServo extends Thread {
    private boolean enabled = false;
    private CRServoImplEx servo;

    private long lastTime;

    private double lastPower = 0;
    private double power = 0;

    private int minTimeBetweenUpdates = 0;

    public BetterThreadedCRServo(CRServoImplEx servo, int minTimeBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    @Override
    public void run() {
        while (enabled) {
            updateSetPower(power);

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
        if (currentTime - lastTime >= minTimeBetweenUpdates && power != lastPower) {
            servo.setPower(power);
            lastTime = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void setPower(double power) {
        this.power = power;
    }

    public synchronized void setDirection(CRServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPower() {
        return this.power;
    }

    public void enable() {
        enabled = true;   
    }

    public void disable() {
        enabled = false;
    }
}
