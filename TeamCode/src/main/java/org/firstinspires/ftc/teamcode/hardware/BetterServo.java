package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

// Warning the Axon server is servo mode do not go to the right position all the time when using degrees / 360.
// I need to write a manual interpolation function that we can then use the better servo to put the degrees and it will get close.
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
        if (currentTime - lastTime >= minTimeBetweenUpdates && position != lastPosition) {
            servo.setPosition(position);
            lastTime = System.currentTimeMillis();
            this.lastPosition = position;
        }
    }

    public void updateSetPositionDegrees(double positionDegrees) {
        this.positionDegrees = positionDegrees;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTimeDegrees >= minTimeBetweenUpdates && positionDegrees != lastPositionDegrees) {
            // This is where the interpolate happens
            servo.setPosition(positionDegrees);
            lastTimeDegrees = System.currentTimeMillis();
            this.lastPositionDegrees = positionDegrees;
        }
    }

    public void setDirection(ServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPosition() {
        return this.position;
    }

    public double getPositionDegrees() {
        return this.positionDegrees;
    }
}
