package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedMotors {
    private final DcMotorImplEx masterMotor;
    private final List<DcMotorImplEx> slaveMotors;

    public LinkedMotors(DcMotorImplEx masterMotor, DcMotorImplEx[] slaveMotors) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>(Arrays.asList(slaveMotors));
    }

    public LinkedMotors(DcMotorImplEx masterMotor, DcMotorImplEx slaveMotor) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>();
        this.slaveMotors.add(slaveMotor);
    }

    public void setPower(double power) {
        masterMotor.setPower(power);
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            slaveMotor.setPower(power);
        }
    }

    public void setVelocity(double velocity) {
        masterMotor.setVelocity(velocity);
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            slaveMotor.setVelocity(velocity);
        }
    }

    // Get the power output of only the master motor.
    public double getPower() {
        return masterMotor.getPower();
    }
    
    // Get the rough average power output of all motors that are linked.
    public double getAveragePower() {
        double totalPower;
        totalPower = masterMotor.getPower();
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            totalPower += slaveMotor.getPower();
        }
        int motorCount = 1 + slaveMotors.size();
        return totalPower / motorCount;
    }

    public void setDirection(DcMotorImplEx.Direction direction) {
        masterMotor.setDirection(direction);
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            slaveMotor.setDirection(direction);
        }
    }

    public void setMode(DcMotorImplEx.RunMode runMode) {
        masterMotor.setMode(runMode);
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            slaveMotor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior behavior) {
        masterMotor.setZeroPowerBehavior(behavior);
        for (DcMotorImplEx slaveMotor : slaveMotors) {
            slaveMotor.setZeroPowerBehavior(behavior);
        }
    }
}
