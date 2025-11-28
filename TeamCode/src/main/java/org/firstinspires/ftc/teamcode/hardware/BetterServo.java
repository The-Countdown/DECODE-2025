package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterServo {
    private ServoImplEx servo;

    private long lastTime;
    private long lastTimeDegrees;

    private double lastPosition = 0;
    private double position = 0;
    private double lastPositionDegrees = 0;
    private double positionDegrees = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterServo(ServoImplEx servo, int minTimeBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public void updateSetPosition(double position) {
        this.position = position;
        long currentTime = System.currentTimeMillis();
        if (currentTime - this.lastTime >= this.minTimeBetweenUpdates && this.position != this.lastPosition) {
            servo.setPosition(this.position);
            lastTime = System.currentTimeMillis();
            this.lastPosition = this.position;
        }
    }

    // Note that the max range of a servo is 0-322 degrees
    public void updateSetPositionDegrees(double positionDegrees) {
        this.positionDegrees = ((HelperFunctions.normalizeAngle(positionDegrees) + 180) / 322);
        long currentTime = System.currentTimeMillis();
        if (currentTime - this.lastTimeDegrees >= this.minTimeBetweenUpdates && this.positionDegrees != this.lastPositionDegrees) {
            // This is where the interpolate happens
            servo.setPosition(this.positionDegrees);
            this.lastTimeDegrees = System.currentTimeMillis();
            this.lastPositionDegrees = this.positionDegrees;
        }
    }

    public void setDirection(ServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPosition() {
        return this.position;
    }

    public double getPositionDegrees() {
        return (this.positionDegrees * 322);
    }
}
