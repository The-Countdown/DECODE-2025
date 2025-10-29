package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Transfer extends RobotContainer.HardwareDevices {
    private RobotContainer robotContainer;
    private final ServoImplEx transferServoRight;
    private final ServoImplEx transferServoLeft;

    public Transfer(RobotContainer robotContainer, ServoImplEx transferServoRight, ServoImplEx transferServoLeft) {
        this.robotContainer = robotContainer;
        this.transferServoRight = transferServoRight;
        this.transferServoLeft = transferServoLeft;
    }

    public void zeroServo() {
        transferServoLeft.setPosition(0);
        transferServoRight.setPosition(0);
    }
    public void flapUp() {
        transferServoRight.setPosition(Constants.Transfer.UP);}
    public void flapDown() {
        transferServoRight.setPosition(Constants.Transfer.DOWN);}
}
