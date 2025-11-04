package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;

import org.firstinspires.ftc.teamcode.main.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedMotors {
    private final BetterDcMotor masterMotor;
    private final List<BetterDcMotor> slaveMotors;

    public LinkedMotors(BetterDcMotor masterMotor, BetterDcMotor[] slaveMotors) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>(Arrays.asList(slaveMotors));
    }

    public LinkedMotors(BetterDcMotor masterMotor, BetterDcMotor slaveMotor) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>();
        this.slaveMotors.add(slaveMotor);
    }

    public void setPower(double power) {
        masterMotor.updateSetPower(power);
        for (BetterDcMotor slaveMotor : slaveMotors) {
            slaveMotor.updateSetPower(power);
        }
    }

    public void setVelocity(double velocity) {
        masterMotor.updateSetVelocity(velocity);
        for (BetterDcMotor slaveMotor : slaveMotors) {
            slaveMotor.updateSetVelocity(velocity);
        }
    }

    public double getVelocity() {
        return masterMotor.getVelocity();
    }

    public double getAverageVelocity() {
        double totalVelocity;
        totalVelocity = masterMotor.getVelocity();
        for (BetterDcMotor slaveMotor : slaveMotors) {
            totalVelocity += slaveMotor.getVelocity();
        }
        int motorCount = 1 + slaveMotors.size();
        return totalVelocity / motorCount;
    }

    // Get the power output of only the master motor.
    public double getPower() {
        return masterMotor.getPower();
    }

    // Get the rough average power output of all motors that are linked.
    public double getAveragePower() {
        double totalPower;
        totalPower = masterMotor.getPower();
        for (BetterDcMotor slaveMotor : slaveMotors) {
            totalPower += slaveMotor.getPower();
        }
        int motorCount = 1 + slaveMotors.size();
        return totalPower / motorCount;
    }

    public void setDirection(DcMotorImplEx.Direction direction) {
        masterMotor.setDirection(direction);
        for (BetterDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setDirection(direction);
        }
    }

    public void setMode(DcMotorImplEx.RunMode runMode) {
        masterMotor.setMode(runMode);
        for (BetterDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior behavior) {
        masterMotor.setZeroPowerBehavior(behavior);
        for (BetterDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setZeroPowerBehavior(behavior);
        }
    }
}
