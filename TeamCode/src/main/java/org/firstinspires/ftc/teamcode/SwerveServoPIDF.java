package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A PIDF controller for a swerve module's servo.
 */
public class SwerveServoPIDF {
    private final RobotManager robotManager;
    private final CRServoImplEx servo;
    private final int module;
    private double targetAngle;
    private final ElapsedTime timer;
    private double lastError = 0;
    public double p;
    public double i;
    public double d;
    public double ff;

    public SwerveServoPIDF(RobotManager robotManager, int module, CRServoImplEx servo) {
        this.robotManager = robotManager;
        this.servo = servo;
        this.module = module;
        this.targetAngle = Constants.SWERVE_STOP_FORMATION[module];
        this.timer = new ElapsedTime();
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        double error = robotManager.swerveModules[module].servo.getAngle() - robotManager.swerveServosPIDF[module].getTargetAngle();
        error = robotManager.drivetrain.normalizeAngle(error);

        return error;
    }

    /**
     * Calculates the PIDF output for the servo.
     * @return The calculated PIDF output.
     */
    public double calculate() {
        double currentAngle = robotManager.swerveModules[module].servo.getAngle();
        double error = targetAngle - currentAngle;
        double currentTime = timer.seconds();
        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        p = Constants.SWERVE_SERVO_KP * error;
        i += Constants.SWERVE_SERVO_KI * error * currentTime;
        d = Constants.SWERVE_SERVO_KD * (error - lastError) / currentTime;
        ff = Constants.SWERVE_SERVO_KF * Math.signum(error);

        lastError = error;

        return p + i + d + ff;
    }
}
