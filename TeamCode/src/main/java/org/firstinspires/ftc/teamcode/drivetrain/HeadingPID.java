package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

/**
 * A PID controller for maintaining a specific heading.
 */
public class HeadingPID {
    private final RobotContainer robotContainer;
    private double targetHeading = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double p;
    private double i;
    private double d;

    public HeadingPID(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Calculates the PID output for the servo.
     * @param heading The current heading of the robot.
     * @return The calculated PID output.
     */
    public double calculate(double heading) {
        double error = robotContainer.drivetrain.normalizeAngle(targetHeading - heading);

        double currentTime = timer.seconds();
        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        if (Math.abs(error) < Constants.HEADING_PID_TOLERANCE_DEGREES) {
            lastError = error;
            Status.robotHeadingTargetReached = true;
            return 0;
        } else {
            Status.robotHeadingTargetReached = false;
        }

        p = Constants.HEADING_KP * error;
        double newI = Math.max(-Constants.HEADING_I_MAX, Math.min(Constants.HEADING_I_MAX,  // Prevent integral windup
                i + Constants.HEADING_KI * error * currentTime));

            if (Math.abs(p + newI + d) < 1) {
                i = newI; // Only allow integration if output is within limits
            }

            // Decay integral if error changes sign
            if (Math.signum(error) != Math.signum(lastError)) {
                i *= 0.9;
            }
        d = Constants.HEADING_KD * (error - lastError) / currentTime;

        lastError = error;

        return p + i + d;
    }
}