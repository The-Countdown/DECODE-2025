package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A PID controller for maintaining a specific heading.
 */
public class HeadingHoldPID {
    private final RobotManager robotManager;
    private double targetHeading = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double p;
    private double i;
    private double d;

    HeadingHoldPID(RobotManager robotManager) {
        this.robotManager = robotManager;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    /**
     * Calculates the PID output for the servo.
     * @param heading The current heading of the robot.
     * @return The calculated PID output.
     */
    public double calculate(double heading) {
        double error = targetHeading - heading;
        double currentTime = timer.seconds();
        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        p = Constants.HEADING_KP * error;
        i += Constants.HEADING_KI * error * currentTime;
        d = Constants.HEADING_KD * (error - lastError) / currentTime;

        lastError = error;

        return p + i + d;
    }
}