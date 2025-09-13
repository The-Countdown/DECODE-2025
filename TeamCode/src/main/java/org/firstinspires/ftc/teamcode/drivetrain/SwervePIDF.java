package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

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
    private double lastAngle = 0;
    private double lastTime = 0;
    public double p;
    public double i;
    public double d;
    public double swerveConstantPower;
    public double ff;

    public SwervePIDF(RobotContainer robotContainer, int module, CRServoImplEx servo) {
        this.robotContainer = robotContainer;
        this.servo = servo;
        this.module = module;
        this.targetAngle = Constants.SWERVE_STOP_FORMATION[module];
        this.timer = new ElapsedTime();
    }

    public void setTargetAngle(double angle) {
        targetAngle = HelperFunctions.normalizeAngle(angle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        double error = targetAngle - robotContainer.swerveModules[module].servo.getAngle();
        error = HelperFunctions.normalizeAngle(error);

        return error;
    }

    /**
     * Calculates the PIDF output for the servo.
     * @return The calculated PIDF output.
     */
    public double calculate() {
        double error = getError();
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt < 1e-4) dt = 1e-4;  // safety clamp

        p = Constants.SWERVE_SERVO_KP[module] * error;

        i += Constants.SWERVE_SERVO_KI[module] * error * currentTime;
        i = Math.max(-Constants.SWERVE_SERVO_I_MAX[module], Math.min(Constants.SWERVE_SERVO_I_MAX[module], i));

        double angle = robotContainer.swerveModules[module].servo.getAngle();
        double velocity = (angle - lastAngle) / dt;
        d = -Constants.SWERVE_SERVO_KD[module] * velocity;

        swerveConstantPower = (Constants.SWERVE_SERVO_KF[module] * (1 - (robotContainer.swerveModules[module].motor.targetPower * Constants.SWERVE_SERVO_MOTOR_VELOCITY[module])));
        if (swerveConstantPower < 0) {
            swerveConstantPower = 0;
        }
        ff = swerveConstantPower * Math.signum(error);

        lastError = error;
        lastAngle = angle;

        return p + i + d + ff;
    }
}
