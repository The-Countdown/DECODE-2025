package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.teamcode.hardware.BetterServo;

public class Transfer extends RobotContainer.HardwareDevices {
    private RobotContainer robotContainer;
    private final BetterServo transferServoRight;
    private final BetterServo transferServoLeft;

    public Transfer(RobotContainer robotContainer, BetterServo transferServoRight, BetterServo transferServoLeft) {
        this.robotContainer = robotContainer;
        this.transferServoRight = transferServoRight;
        this.transferServoLeft = transferServoLeft;
    }

    public void zeroServo() {
        transferServoLeft.updateSetPosition(0, Constants.Robot.SERVO_UPDATE_TIME);
        transferServoRight.updateSetPosition(0, Constants.Robot.SERVO_UPDATE_TIME);
    }
    public void flapUp() {
        transferServoRight.updateSetPosition(Constants.Transfer.UP, Constants.Robot.SERVO_UPDATE_TIME);
        transferServoLeft.updateSetPosition(Constants.Transfer.UP, Constants.Robot.SERVO_UPDATE_TIME);
    }

    public void flapDown() {
        transferServoRight.updateSetPosition(Constants.Transfer.DOWN, Constants.Robot.SERVO_UPDATE_TIME);
        transferServoLeft.updateSetPosition(Constants.Transfer.DOWN, Constants.Robot.SERVO_UPDATE_TIME);
    }
}
