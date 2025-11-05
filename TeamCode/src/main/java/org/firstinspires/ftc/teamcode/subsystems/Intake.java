package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.teamcode.hardware.BetterThreadedDcMotor;

public class Intake {
    private RobotContainer robotContainer;
    private BetterThreadedDcMotor intakeMotor;


    public Intake(RobotContainer robotContainer, BetterThreadedDcMotor intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;
    }

    public void setVelocity(double power) {
        intakeMotor.setVelocity(Constants.Swerve.MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}
