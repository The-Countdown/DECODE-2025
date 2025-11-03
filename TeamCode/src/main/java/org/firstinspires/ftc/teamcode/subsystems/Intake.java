package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;

public class Intake {
    private RobotContainer robotContainer;
    private BetterDcMotor intakeMotor;


    public Intake(RobotContainer robotContainer, BetterDcMotor intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;
    }

    public void setVelocity(double power) {
        intakeMotor.updateSetVelocity(Constants.Swerve.MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power, Constants.Robot.MOTOR_UPDATE_TIME);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}
