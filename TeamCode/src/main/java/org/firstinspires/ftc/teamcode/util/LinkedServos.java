package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;

import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedServos {
    private BetterServo masterServo;
    private List<BetterServo> slaveServo;
    private BetterCRServo masterCRServo;
    private List<BetterCRServo> slaveCRServo;

    public LinkedServos(BetterServo masterServo, BetterServo[] slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    public LinkedServos(BetterServo masterServo, BetterServo slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>();
        this.slaveServo.add(slaveServo);
    }

    public LinkedServos(BetterCRServo masterServo, BetterCRServo[] slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    public LinkedServos(BetterCRServo masterServo, BetterCRServo slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>();
        this.slaveCRServo.add(slaveServo);
    }

    public void setPosition(double position) {
        masterServo.updateSetPosition(position);
        for (BetterServo slaveServo : slaveServo) {
            slaveServo.updateSetPosition(position);
        }
    }

    public void setPositionDegrees(double degree) {
        masterServo.updateSetPositionDegrees(degree);
        for (BetterServo slaveServo : slaveServo) {
            slaveServo.updateSetPositionDegrees(degree);
        }
    }

    //set power 1 -1
    public void setPower(double power) {
        masterCRServo.updateSetPower(power);
        for (BetterCRServo slaveCRServo : slaveCRServo) {
            slaveCRServo.updateSetPower(power);
        }
    }
    public double getPosition() {
        return masterServo.getPosition();
    }

    public double getPositionDegrees() {
        return masterServo.getPositionDegrees();
    }
}
