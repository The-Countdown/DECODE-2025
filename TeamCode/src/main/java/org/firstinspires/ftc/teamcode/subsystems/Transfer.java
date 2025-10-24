package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Transfer extends RobotContainer.HardwareDevices {
    private RobotContainer robotContainer;
    private final ServoImplEx transferServoLow;
    private final CRServoImplEx transferServoHigh;

    public Transfer(RobotContainer robotContainer, ServoImplEx transferServoLow, CRServoImplEx transferServoHigh) {
        this.robotContainer = robotContainer;
        this.transferServoLow = transferServoLow;
        this.transferServoHigh = transferServoHigh;
    }

//    public void setLowPower(double power) {
//        transferServoLow.setPower(power);
//    }

    public void setHighPower(double power) {
        transferServoHigh.setPower(power);
    }

    public void flapUp() {transferServoLow.setPosition(Constants.Transfer.UP);}
    public void flapDown() {transferServoLow.setPosition(Constants.Transfer.DOWN);}
}
