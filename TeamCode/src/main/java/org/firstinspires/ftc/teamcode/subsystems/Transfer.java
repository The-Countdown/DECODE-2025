package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

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
        transferServoLeft.updateSetPosition(0);
        transferServoRight.updateSetPosition(0);
    }
    public void flapUp() {
        transferServoRight.updateSetPosition(Constants.Transfer.UP);
        transferServoLeft.updateSetPosition(Constants.Transfer.UP);
    }

    public void flapDown() {
        transferServoRight.updateSetPosition(Constants.Transfer.DOWN);
        transferServoLeft.updateSetPosition(Constants.Transfer.DOWN);
    }
}
