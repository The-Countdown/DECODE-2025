package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

/**
 * A PIDF controller for a swerve module's servo.
 */
public class SwervePIDF {
    private final RobotContainer robotContainer;
    private final CRServoImplEx servo;
    private final int module;
    private double targetAngle;
    private final ElapsedTime timer;
    private double lastError = 0;
    public double p;
    public double i;
    public double d;
    public double ff;

    public SwervePIDF(RobotContainer robotContainer, int module, CRServoImplEx servo) {
        this.robotContainer = robotContainer;
        this.servo = servo;
        this.module = module;
        this.targetAngle = Constants.SWERVE_STOP_FORMATION[module];
        this.timer = new ElapsedTime();
    }

    public void setTargetAngle(double angle) {
        targetAngle = robotContainer.drivetrain.normalizeAngle(angle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        double error = robotContainer.swerveModules[module].servo.getAngle() - robotContainer.swerveServosPIDF[module].getTargetAngle();
        error = robotContainer.drivetrain.normalizeAngle(error);

        return error;
    }

    /**
     * Calculates the PIDF output for the servo.
     * @return The calculated PIDF output.
     */
    public double calculate() {
        double currentAngle = robotContainer.swerveModules[module].servo.getAngle();
        double error = robotContainer.drivetrain.normalizeAngle(targetAngle - currentAngle);
        double currentTime = timer.seconds();
        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        p = Constants.SWERVE_SERVO_KP * error;
        double newI = Math.max(-Constants.SWERVE_SERVO_I_MAX, Math.min(Constants.SWERVE_SERVO_I_MAX,  // Prevent integral windup
                i + Constants.SWERVE_SERVO_KI * error * currentTime));

            if (Math.abs(p + newI + d) < 1) {
                i = newI; // Only allow integration if output is within limits
            }

            // Decay integral if error changes sign
            if (Math.signum(error) != Math.signum(lastError)) {
                i *= 0.9;
            }
        d = Constants.SWERVE_SERVO_KD * (error - lastError) / currentTime;
        ff = Constants.SWERVE_SERVO_KF * Math.signum(error);

        lastError = error;

        return p + i + d + ff;
    }
}
