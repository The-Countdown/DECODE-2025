package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveServoPIDF {
    private Robot robot;
    private CRServoImplEx servo;
    private int module;
    private double targetAngle;
    private ElapsedTime timer;
    private double integral, lastError;
    private double p;
    private double i;
    private double d;
    private double ff;

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
