package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class LinkedMotor {
    private final DcMotorImplEx masterMotor;
    private final DcMotorImplEx slaveMotor;

    public LinkedMotor(DcMotorImplEx masterMotor, DcMotorImplEx slaveMotor) {
        this.masterMotor = masterMotor;
        this.slaveMotor = slaveMotor;
    }

    public void setPower(double power) {
        masterMotor.setPower(power);
        slaveMotor.setPower(power);
    }

    public double getPower() { return masterMotor.getPower(); }

    public void setDirection(DcMotorImplEx.Direction direction) {
        masterMotor.setDirection(direction);
        slaveMotor.setDirection(direction);
    }

    public void setMode(DcMotorImplEx.RunMode runMode) {
        masterMotor.setMode(runMode);
        slaveMotor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior behavior) {
        masterMotor.setZeroPowerBehavior(behavior);
        slaveMotor.setZeroPowerBehavior(behavior);
    }
}