package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.teamcode.hardware.BetterThreadedServo;

public class Transfer extends RobotContainer.HardwareDevices {
    private RobotContainer robotContainer;
    private final BetterThreadedServo transferServoRight;
    private final BetterThreadedServo transferServoLeft;

    public Transfer(RobotContainer robotContainer, BetterThreadedServo transferServoRight, BetterThreadedServo transferServoLeft) {
        this.robotContainer = robotContainer;
        this.transferServoRight = transferServoRight;
        this.transferServoLeft = transferServoLeft;
    }

    public void zeroServo() {
        transferServoLeft.setPosition(0);
        transferServoRight.setPosition(0);
    }
    public void flapUp() {
        transferServoRight.setPosition(Constants.Transfer.UP);
        transferServoLeft.setPosition(Constants.Transfer.UP);
    }

    public void flapDown() {
        transferServoRight.setPosition(Constants.Transfer.DOWN);
        transferServoLeft.setPosition(Constants.Transfer.DOWN);
    }
}
