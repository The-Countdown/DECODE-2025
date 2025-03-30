package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A PIDF controller for a swerve module's servo.
 */
public class SwerveServoPIDF {
    private final Robot robot;
    private final CRServoImplEx servo;
    private final int module;
    private double targetAngle;
    private final ElapsedTime timer;
    private double integral;
    private double lastError;
    public double p;
    public double i;
    public double d;
    public double ff;

    public SwerveServoPIDF(Robot robot, int module, CRServoImplEx servo) {
        this.robot = robot;
        this.servo = servo;
        this.module = module;
        this.targetAngle = Constants.SWERVE_STOP_FORMATION[module];
        this.timer = new ElapsedTime();
        this.integral = 0;
        this.lastError = 0;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        double error = robot.swerveModules[module].servo.getAngle() - robot.swerveServosPIDF[module].getTargetAngle();
        error = robot.drivetrain.normalizeAngle(error);

        return error;
    }

    /**
     * Calculates the PIDF output for the servo.
     * @return The calculated PIDF output.
     */
    public double calculate() {
        double currentAngle = robot.swerveModules[module].servo.getAngle();
        double error = targetAngle - currentAngle;
        double currentTime = timer.seconds();
        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        p = Constants.kP * error;
        i += Constants.kI * error * currentTime;
        d = (error - lastError) / currentTime;
        ff = Constants.kF * Math.signum(error);

        lastError = error;

        return p + i + d + ff;
    }
}
