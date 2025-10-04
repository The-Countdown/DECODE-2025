package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedServos {
    private ServoImplEx masterServo;
    private List<ServoImplEx> slaveServo;
    private CRServoImplEx masterCRServo;
    private List<CRServoImplEx> slaveCRServo;

    //Servo, take in array
    public LinkedServos(ServoImplEx masterServo, ServoImplEx[] slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    // Servo, take in single
    public LinkedServos(ServoImplEx masterServo, ServoImplEx slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>();
        this.slaveServo.add(slaveServo);
    }

    //CRServo, take in array
    public LinkedServos(CRServoImplEx masterServo, CRServoImplEx[] slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    //CRServo, take in single
    public LinkedServos(CRServoImplEx masterServo, CRServoImplEx slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>();
        this.slaveCRServo.add(slaveServo);
    }

    //set pos 1 to -1
    public void setPosition(double position) {
        masterServo.setPosition(position);
        for (ServoImplEx slaveServo : slaveServo) {
            slaveServo.setPosition(position);
        }
    }

    //set pos degree
    public void setPositionDegree(double degree) {
        degree = HelperFunctions.normalizeAngle(degree) + 180;
        degree = degree > 355 ? 355 : degree; // if degree > 355 ? condition is true, return 355, otherwise(:) return degree
        masterServo.setPosition(degree/355);
        for (ServoImplEx slaveServo : slaveServo) {
            slaveServo.setPosition(degree/355);
        }
    }

    //set power 1 -1
    public void setPower(double power) {
        masterCRServo.setPower(power);
        for (CRServoImplEx slaveCRServo : slaveCRServo) {
            slaveCRServo.setPower(power);
        }
    }
}
