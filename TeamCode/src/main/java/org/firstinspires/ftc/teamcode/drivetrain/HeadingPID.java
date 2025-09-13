package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

/**
 * A PID controller for maintaining a specific heading.
 */
public class HeadingPID {
    private final RobotContainer robotContainer;
    private boolean enabled = true;
    private double targetHeading = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double currentTime;
    private double lastTime;
    private double p;
    private double i;
    private double d;
    private double f;

    public HeadingPID(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
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
        if (enabled) {
            double error = HelperFunctions.normalizeAngle(targetHeading - heading);
            currentTime = timer.seconds();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            timer.reset();
            if (currentTime < 1e-6) currentTime = 1e-6;

            if (Math.abs(error) < Constants.HEADING_PID_TOLERANCE_DEGREES) {
                lastError = error;
                Status.robotHeadingTargetReached = true;
                return 0;
            } else {
                Status.robotHeadingTargetReached = false;
            }

            if (dt < 1e-4) dt = 1e-4;  // safety clamp

            p = Constants.HEADING_KP * error;

            i += Constants.HEADING_KI * error * currentTime;
            i = Math.max(-Constants.HEADING_I_MAX, Math.min(Constants.HEADING_I_MAX, i));

            d = -Constants.HEADING_KD * (error - lastError) / currentTime;

            lastError = error;

            return p + i + d + Constants.HEADING_KF;
        }
        else {
            return 0;
        }
    }
}
