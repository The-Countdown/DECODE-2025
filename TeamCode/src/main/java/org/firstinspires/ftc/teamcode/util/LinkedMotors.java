package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.hardware.BetterThreadedDcMotor;

import org.firstinspires.ftc.teamcode.main.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedMotors {
    private final BetterThreadedDcMotor masterMotor;
    private final List<BetterThreadedDcMotor> slaveMotors;

    public LinkedMotors(BetterThreadedDcMotor masterMotor, BetterThreadedDcMotor[] slaveMotors) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>(Arrays.asList(slaveMotors));
    }

    public LinkedMotors(BetterThreadedDcMotor masterMotor, BetterThreadedDcMotor slaveMotor) {
        this.masterMotor = masterMotor;
        this.slaveMotors = new ArrayList<>();
        this.slaveMotors.add(slaveMotor);
    }

    public void setPower(double power) {
        masterMotor.setPower(power);
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setPower(power);
        }
    }

    public void setVelocity(double velocity) {
        masterMotor.setVelocity(velocity);
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setVelocity(velocity);
        }
    }

    public double getVelocity() {
        return masterMotor.getVelocity();
    }

    public double getAverageVelocity() {
        double totalVelocity;
        totalVelocity = masterMotor.getVelocity();
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
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
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            totalPower += slaveMotor.getPower();
        }
        int motorCount = 1 + slaveMotors.size();
        return totalPower / motorCount;
    }

    public void setDirection(DcMotorImplEx.Direction direction) {
        masterMotor.setDirection(direction);
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setDirection(direction);
        }
    }

    public void setMode(DcMotorImplEx.RunMode runMode) {
        masterMotor.setMode(runMode);
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior behavior) {
        masterMotor.setZeroPowerBehavior(behavior);
        for (BetterThreadedDcMotor slaveMotor : slaveMotors) {
            slaveMotor.setZeroPowerBehavior(behavior);
        }
    }
}
