package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Transfer extends RobotContainer.HardwareDevices {
    private RobotContainer robotContainer;
    private final CRServoImplEx transferServoLow;
    private final CRServoImplEx transferServoHigh;

    public Transfer(RobotContainer robotContainer, CRServoImplEx transferServoLow, CRServoImplEx transferServoHigh) {
        this.robotContainer = robotContainer;
        this.transferServoLow = transferServoLow;
        this.transferServoHigh = transferServoHigh;
    }

    public void setLowPower(double power) {
        transferServoLow.setPower(power);
    }

    public void setHighPower(double power) {
        transferServoHigh.setPower(power);
    }
}
