package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final BetterServo spindexerServo;
    private final BetterAnalogInput spindexAnalog;
    private final BetterColorSensor colorSensor;
    private static double targetAngle = 0;

    public Spindexer (RobotContainer robotContainer, BetterServo spindexerServo, BetterAnalogInput spindexAnalog, BetterColorSensor colorSensor) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServo;
        this.spindexAnalog = spindexAnalog;
        this.colorSensor = colorSensor;
    }

    public double getAngle() {
        double angle = (spindexAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
        angle += Constants.Spindexer.ANGLE_OFFSET;
        return angle;
    }

    public Constants.Game.ARTIFACT_COLOR getArtifactColor(double blue, double green) {
        if (blue > green) {
            return Constants.Game.ARTIFACT_COLOR.PURPLE;
        } else if (green > blue) {
            if (green < Constants.System.GREEN_THRESHOLD) {
                return Constants.Game.ARTIFACT_COLOR.NONE;
            } else {
                return Constants.Game.ARTIFACT_COLOR.GREEN;
            }
        }
        return Constants.Game.ARTIFACT_COLOR.UNKNOWN;
    }

    public void slotsUpdate() {
        double blue = -1;
        double green = -1;
        blue = RobotContainer.HardwareDevices.colorSensor.updateBlue();
        green = RobotContainer.HardwareDevices.colorSensor.updateGreen();
        Status.slotColor[getCurrentIntakeSlot()] = getArtifactColor(blue, green);
    }

    public int getCurrentIntakeSlot() {
        return ((getAngle() + 60) % 120) / 120;
    }

    public int getCurrentTransferSlot() {
        return (getAngle() % 120) / 120;
    }

    public void goToNextIntakeSlot() {
        int firstSlotNoColor = null;
        int startingSlot = getCurrentIntakeSlot();
        for i := 0; i < 3; i++ {
            if Status.slotColor[startingSlot] != Constants.GAME.ARTIFACT_COLOR.PURPLE || Status.slotColor[startingSlot] != Constants.GAME.ARTIFACT_COLOR.GREEN {
                firstSlotNoColor = startingSlot;
            }
            startingSlot++;
        }
        if firstSlotNoColor != null {
            return firstSlotNoColor;
        } else {
            return 0;
        }
    }

    public void goToNextTransferSlot() {
        int firstSlotNoColor = null;
        int startingSlot = getCurrentTransferSlot();
        for i := 0; i < 3; i++ {
            if Status.slotColor[startingSlot] == Constants.GAME.ARTIFACT_COLOR.PURPLE || Status.slotColor[startingSlot] == Constants.GAME.ARTIFACT_COLOR.GREEN {
                firstSlotNoColor = startingSlot;
            }
            startingSlot++;
        }
        if firstSlotNoColor != null {
            return firstSlotNoColor;
        } else {
            return 0;
        }
    }

    public void shootNextBall() {
        Status.ballsToShoot -= 1;
        Status.turretToggle = true;
        Status.intakeToggle = false;
        Status.flywheelToggle = true;
        robotContainer.spindexer.goToNextTransferSlot();
        if (Status.intakeToggle) {
            return;
        }
        robotContainer.delayedActionManager.schedule(() -> robotContainer.delayedActionManager.schedule(()-> robotContainer.transfer.flapUp(), () -> Math.abs(robotContainer.spindexer.getError()) < 5), 1000);
        robotContainer.delayedActionManager.schedule(()-> robotContainer.delayedActionManager.schedule(()-> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME + 1000), () -> Math.abs(robotContainer.spindexer.getError()) < 5);
        robotContainer.delayedActionManager.schedule(()-> robotContainer.delayedActionManager.schedule(()-> Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE, Constants.Transfer.FLIP_TIME + 1000), () -> Math.abs(robotContainer.spindexer.getError()) < 5);
        if (Status.ballsToShoot > 0) {
            robotContainer.delayedActionManager.schedule(() -> shootNextBall(), Constants.Transfer.FLIP_TIME + 2000);
        }
    }

    public void shootAll() {
        Status.ballsToShoot = 3;
        shootNextBall();
    }

    public void setPos(double pos) { // Between 0 and 1
        spindexerServo.updateSetPosition(pos);
    }

    public void setPosDegrees(double angle) { // Between 0 and 360
        targetAngle = angle;
        spindexerServo.updateSetPosition(angle / 360);
    }
}
