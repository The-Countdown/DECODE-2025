package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Intake {
    private RobotContainer robotContainer;
    private BetterDcMotor intakeMotor;


    public Intake(RobotContainer robotContainer, BetterDcMotor intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;

        intakeMotor.start(); // Remember to stop this at some point
    }

    public void setPower(double power) {
        intakeMotor.updateSetPower(power);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getPower() {
        return intakeMotor.getPower();
    }
}
