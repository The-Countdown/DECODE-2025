package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

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
    public double targetAngle = 0;
    public double lastPosition = 0;

    public Spindexer (RobotContainer robotContainer, BetterServo spindexerServo, BetterAnalogInput spindexAnalog, BetterColorSensor colorSensor) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServo;
        this.spindexAnalog = spindexAnalog;
        this.colorSensor = colorSensor;
        this.lastPosition = getRawAngle();
    }

    public double getAngle() {
        double angle = (spindexAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
        angle += Constants.Spindexer.ANGLE_OFFSET;
        return angle;
    }

    public double getRawAngle() {
        return (spindexAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
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

    public void slotUpdate() {
        // double blue = -1;
        // double green = -1;
        // blue = RobotContainer.HardwareDevices.colorSensor.updateBlue();
        // green = RobotContainer.HardwareDevices.colorSensor.updateGreen();
        // Status.slotColor[getCurrentIntakeSlot()] = getArtifactColor(blue, green);

        if (!jam()) {
            Status.slotColor[getCurrentIntakeSlot() % 3] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        } else {
            int currentSlot = getCurrentIntakeSlot();
            if (currentSlot == 0) {
                currentSlot = 2;
            }
            currentSlot -= 1;
            targetAngle = Constants.Spindexer.INTAKE_SLOT_ANGLES[currentSlot];
            spindexerServo.updateSetPosition(targetAngle);
            int finalCurrentSlot = currentSlot;
            robotContainer.delayedActionManager.schedule(() -> spindexerServo.updateSetPosition(finalCurrentSlot), 300);
        }
    }

    // Assume the Spindexer is always at target
    public int getCurrentIntakeSlot() {
        int current = (int) (((targetAngle % 360) + 60) / 120);
        robotContainer.telemetry.addData("current intake", current);
        return current % 3;
    }

    // Assume the Spindexer is always at target
    public int getCurrentTransferSlot() {
        int current = (int) (targetAngle % 360) / 120;
        robotContainer.telemetry.addData("current transfer", current);
        return current % 3;
    }

    public void goToNextIntakeSlot() {
        int firstSlotNoColor = -1;
        int currentSlot = getCurrentIntakeSlot();
        for (int i = 0; i < 3; i++) {
            if (Status.slotColor[currentSlot % 3] != Constants.Game.ARTIFACT_COLOR.PURPLE && Status.slotColor[currentSlot % 3] != Constants.Game.ARTIFACT_COLOR.GREEN) {
                firstSlotNoColor = (currentSlot % 3);
                break;
            }
            currentSlot++;
        }
        if (firstSlotNoColor != -1) {
            setPosDegrees(Constants.Spindexer.INTAKE_SLOT_ANGLES[firstSlotNoColor]);
            robotContainer.telemetry.addData("Next Spindexer ange", Constants.Spindexer.INTAKE_SLOT_ANGLES[firstSlotNoColor]);
        }
    }

    public void goToNextTransferSlot() {
        int firstSlotNoColor = -1;
        int currentSlot = getCurrentTransferSlot();
        for (int i = 0; i < 3; i++) {
            if (Status.slotColor[currentSlot % 3] == Constants.Game.ARTIFACT_COLOR.PURPLE || Status.slotColor[currentSlot % 3] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                firstSlotNoColor = (currentSlot % 3);
            }
            currentSlot++;
        }
        if (firstSlotNoColor != -1) {
            setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[firstSlotNoColor]);
            robotContainer.telemetry.addData("Next Spindexer ange", Constants.Spindexer.TRANSFER_SLOT_ANGLES[firstSlotNoColor]);
        }
    }

    public void goToNextPurpleSlot() {
        int firstSlotPurple = -1;
        int currentSlot = getCurrentTransferSlot();
        for (int i = 0; i < 3; i++) {
            if (Status.slotColor[currentSlot] == Constants.Game.ARTIFACT_COLOR.PURPLE) {
                firstSlotPurple = (currentSlot % 3);
            }
            currentSlot++;
        }
        if (firstSlotPurple != -1) {
            setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[firstSlotPurple] + 60);
        }
    }

    public void goToNextGreenSlot() {
        int firstSlotGreen = -1;
        int currentSlot = getCurrentTransferSlot();
        for (int i = 0; i < 3; i++) {
            if (Status.slotColor[currentSlot] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                firstSlotGreen = (currentSlot % 3);
            }
            currentSlot++;
        }
        if (firstSlotGreen != -1) {
            setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[firstSlotGreen] + 60);
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

    public boolean isFull() {
        int balls = 0;
        for (int i = 0; i < Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
            if (Status.slotColor[i] == Constants.Game.ARTIFACT_COLOR.PURPLE || Status.slotColor[i] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                balls += 1;
            }
        }
        if (balls == 3) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isEmpty() {
        int balls = 0;
        for (int i = 0; i < Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
            if (Status.slotColor[i] == Constants.Game.ARTIFACT_COLOR.PURPLE || Status.slotColor[i] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                balls += 1;
            }
        }
        if (balls > 0) {
            return false;
        } else {
            return true;
        }
    }

    public boolean jam() {
        double position = getRawAngle();
        if (Math.abs(position - lastPosition) < Constants.Spindexer.JAM_ANGLE && robotContainer.beamBreakToggleButton.isPressed()) {
            this.lastPosition = position;
            return true;
        } else {
            this.lastPosition = position;
            return false;
        }
    }

    public double getError() {
        return getAngle() - (targetAngle + (Constants.Spindexer.ANGLE_OFFSET - 17));
    }

    @Deprecated
    public void setPos(double pos) { // Between 0 and 1
        spindexerServo.updateSetPosition(pos);
    }

    public void setPosDegrees(double angle) { // Between 0 and 360
        if (!jam()) {
            angle = angle + Constants.Spindexer.ANGLE_OFFSET;
            targetAngle = angle;
            double target = angle / 360;
            robotContainer.telemetry.addData("Spin Power target", target);
            spindexerServo.updateSetPosition(target);
        } else {
            int currentSlot = getCurrentIntakeSlot();
            if (currentSlot == 0) {
                currentSlot = 2;
            }
            currentSlot -= 1;
            targetAngle = Constants.Spindexer.INTAKE_SLOT_ANGLES[currentSlot];
            spindexerServo.updateSetPosition(targetAngle);
            int finalCurrentSlot = currentSlot;
            robotContainer.delayedActionManager.schedule(() -> spindexerServo.updateSetPosition(finalCurrentSlot), 300);
        }
    }
}
