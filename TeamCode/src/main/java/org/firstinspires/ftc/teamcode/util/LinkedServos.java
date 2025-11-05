package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;

import org.firstinspires.ftc.teamcode.hardware.BetterThreadedCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterThreadedServo;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedServos {
    private BetterThreadedServo masterServo;
    private List<BetterThreadedServo> slaveServo;
    private BetterThreadedCRServo masterCRServo;
    private List<BetterThreadedCRServo> slaveCRServo;

    //Servo, take in array
    public LinkedServos(BetterThreadedServo masterServo, BetterThreadedServo[] slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    // Servo, take in single
    public LinkedServos(BetterThreadedServo masterServo, BetterThreadedServo slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>();
        this.slaveServo.add(slaveServo);
    }

    //CRServo, take in array
    public LinkedServos(BetterThreadedCRServo masterServo, BetterThreadedCRServo[] slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    //CRServo, take in single
    public LinkedServos(BetterThreadedCRServo masterServo, BetterThreadedCRServo slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>();
        this.slaveCRServo.add(slaveServo);
    }

    //set pos 1 to -1
    public void setPosition(double position) {
        masterServo.setPosition(position);
        for (BetterThreadedServo slaveServo : slaveServo) {
            slaveServo.setPosition(position);
        }
    }

    //set pos degree
    public void setPositionDegree(double degree) {
        degree = HelperFunctions.normalizeAngle(degree) + 180;
        degree = degree > 355 ? 355 : degree; // if degree > 355 ? condition is true, return 355, otherwise(:) return degree
        masterServo.setPosition(degree/355);
        for (BetterThreadedServo slaveServo : slaveServo) {
            slaveServo.setPosition(degree/355);
        }
    }

    //set power 1 -1
    public void setPower(double power) {
        masterCRServo.setPower(power);
        for (BetterThreadedCRServo slaveCRServo : slaveCRServo) {
            slaveCRServo.setPower(power);
        }
    }
    public double getPosition() {
        return masterServo.getPosition();
    }
}
