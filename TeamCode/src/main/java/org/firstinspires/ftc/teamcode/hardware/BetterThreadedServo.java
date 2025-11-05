package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterThreadedServo extends Thread {
    private boolean enabled;
    private ServoImplEx servo;

    private long lastTime;

    private double lastPosition = 0;
    private double position = 0;

    private int minTimeBetweenUpdates = 0;

    public BetterThreadedServo(ServoImplEx servo, int minTimeBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    @Override
    public void run() {
        while (enabled) {
            updateSetPosition(position);

            try {
                Thread.sleep(minTimeBetweenUpdates);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }


    public synchronized void updateSetPosition(double position) {
        this.position = position;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates && position != lastPosition) {
            servo.setPosition(position);
            lastTime = System.currentTimeMillis();
            this.lastPosition = position;
        }
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public synchronized void setDirection(ServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPosition() {
        return this.position;
    }

    public void enable() {
        enabled = true;   
    }

    public void disable() {
        enabled = false;
    }
}
