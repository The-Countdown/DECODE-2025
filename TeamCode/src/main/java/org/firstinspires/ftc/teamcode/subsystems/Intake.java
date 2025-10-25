package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Intake {
    private RobotContainer robotContainer;
    private DcMotorImplEx intakeMotor;


    public Intake(RobotContainer robotContainer, DcMotorImplEx intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;
    }

    public void setIntakeVelocity(double power) {
        intakeMotor.setVelocity(Constants.Swerve.MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}
