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
        robotContainer.telemetry.addData("Important angle voltage", angle);

        angle += Constants.Spindexer.ANGLE_OFFSET;

        angle = HelperFunctions.normalizeAngle(angle);

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
        if (robotContainer.spindexer.getAngle() > Constants.Spindexer.INTAKE_SLOT_ANGLES[0] - 10 && robotContainer.spindexer.getAngle() < Constants.Spindexer.INTAKE_SLOT_ANGLES[0] + 10 && (Status.slotColor[0] == Constants.Game.ARTIFACT_COLOR.UNKNOWN || Status.slotColor[0] == Constants.Game.ARTIFACT_COLOR.NONE)) {
            blue = RobotContainer.HardwareDevices.colorSensor.updateBlue();
            green = RobotContainer.HardwareDevices.colorSensor.updateGreen();
            Status.slotColor[0] = getArtifactColor(blue, green);
        }
        if (robotContainer.spindexer.getAngle() > Constants.Spindexer.INTAKE_SLOT_ANGLES[1] - 10 && robotContainer.spindexer.getAngle() < Constants.Spindexer.INTAKE_SLOT_ANGLES[1] + 10 && (Status.slotColor[1] == Constants.Game.ARTIFACT_COLOR.UNKNOWN || Status.slotColor[1] == Constants.Game.ARTIFACT_COLOR.NONE)) {
            if (blue == -1) {
                blue = RobotContainer.HardwareDevices.colorSensor.updateBlue();
                green = RobotContainer.HardwareDevices.colorSensor.updateGreen();
            }
            Status.slotColor[1] = getArtifactColor(blue, green);
        }
        if (robotContainer.spindexer.getAngle() > Constants.Spindexer.INTAKE_SLOT_ANGLES[2] - 10 && robotContainer.spindexer.getAngle() < Constants.Spindexer.INTAKE_SLOT_ANGLES[2] + 10 && (Status.slotColor[2] == Constants.Game.ARTIFACT_COLOR.UNKNOWN || Status.slotColor[2] == Constants.Game.ARTIFACT_COLOR.NONE)) {
            if (green == -1) {
                blue = RobotContainer.HardwareDevices.colorSensor.updateBlue();
                green = RobotContainer.HardwareDevices.colorSensor.updateGreen();
            }
            Status.slotColor[2] = getArtifactColor(blue, green);
        }
    }

    public int getCurrentIntakeSlot() {
        double currentAngle = getAngle();

        int closestIndex = 0;
        double smallestDistance = Math.abs(HelperFunctions.normalizeAngle(currentAngle - Constants.Spindexer.INTAKE_SLOT_ANGLES[0]));
        robotContainer.telemetry.addData("Spindexer index distance", 0);
        robotContainer.telemetry.addData("Spindexer distance", smallestDistance);


        for (int i = 1; i < Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
            double distance = Math.abs(HelperFunctions.normalizeAngle(currentAngle - Constants.Spindexer.INTAKE_SLOT_ANGLES[i]));
            robotContainer.telemetry.addData("Spindexer index distance", i);
            robotContainer.telemetry.addData("Spindexer distance", distance);
            robotContainer.telemetry.addData("Spinder distance compare", Constants.Spindexer.INTAKE_SLOT_ANGLES[i]);
            if (distance <= smallestDistance) {
                smallestDistance = distance;
                robotContainer.telemetry.addData("Set Closest Index", i);
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    public int getCurrentTransferSlot() {
        double currentAngle = getAngle();

        int closestIndex = 0;
        double smallestDistance = Math.abs(HelperFunctions.normalizeAngle(currentAngle - Constants.Spindexer.TRANSFER_SLOT_ANGLES[0]));

        for (int i = 1; i < Constants.Spindexer.TRANSFER_SLOT_ANGLES.length; i++) {
            double distance = Math.abs(HelperFunctions.normalizeAngle(currentAngle - Constants.Spindexer.TRANSFER_SLOT_ANGLES[i]));
            if (distance < smallestDistance) {
                smallestDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    public void goToNextIntakeSlot() {
        robotContainer.spindexer.slotsUpdate();
        boolean intakeSlotFound = false;

        for (int i = 0; i <= Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
            int nextSlot = (robotContainer.spindexer.getCurrentIntakeSlot() + i) % Constants.Spindexer.INTAKE_SLOT_ANGLES.length;
            robotContainer.telemetry.addData("currentSlot" + i, robotContainer.spindexer.getCurrentIntakeSlot());
            robotContainer.telemetry.addData("nextSlot" + i, nextSlot);

            if (Status.slotColor[nextSlot] != Constants.Game.ARTIFACT_COLOR.PURPLE && Status.slotColor[nextSlot] != Constants.Game.ARTIFACT_COLOR.GREEN) {
                robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.setPosDegrees(Constants.Spindexer.INTAKE_SLOT_ANGLES[nextSlot]), Constants.Spindexer.COLOR_SENSE_TIME);
                intakeSlotFound = true;
                break;
            }
        }

        if (!intakeSlotFound) {
            Status.turretToggle = true;
            goToNextTransferSlot();
            Status.intakeToggle = false;
        }
    }

    public void goToNextTransferSlot() {
        boolean turretSlotFound = false;

        for (int i = 0; i <= Constants.Spindexer.TRANSFER_SLOT_ANGLES.length; i++) {
            int nextSlot = (robotContainer.spindexer.getCurrentTransferSlot() + i) % Constants.Spindexer.TRANSFER_SLOT_ANGLES.length;

            if (Status.slotColor[nextSlot] != Constants.Game.ARTIFACT_COLOR.PURPLE && Status.slotColor[nextSlot] != Constants.Game.ARTIFACT_COLOR.GREEN) {
                robotContainer.spindexer.setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[nextSlot]);
                turretSlotFound = true;
                break;
            }
        }

        if (!turretSlotFound) {
            Status.turretToggle = false;
            Status.intakeToggle = true;
            goToNextIntakeSlot();
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

    public void goToNextGreenSlot() {
        boolean turretFound = false;

        for (int i = 1; i <= Constants.Spindexer.TRANSFER_SLOT_ANGLES.length; i++) {
            int nextSlot = (robotContainer.spindexer.getCurrentTransferSlot() + i) % Constants.Spindexer.TRANSFER_SLOT_ANGLES.length;

            if (Status.slotColor[nextSlot] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                robotContainer.spindexer.setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[nextSlot]);
                turretFound = true;
                break;
            }
        }

        if (!turretFound) {
            robotContainer.gamepadEx2.rumble();
        }
    }

    public void goToNextPurpleSlot() {
        boolean turretFound = false;

        for (int i = 1; i <= Constants.Spindexer.TRANSFER_SLOT_ANGLES.length; i++) {
            int nextSlot = (robotContainer.spindexer.getCurrentTransferSlot() + i) % Constants.Spindexer.TRANSFER_SLOT_ANGLES.length;

            if (Status.slotColor[nextSlot] == Constants.Game.ARTIFACT_COLOR.PURPLE) {
                robotContainer.spindexer.setPosDegrees(Constants.Spindexer.TRANSFER_SLOT_ANGLES[nextSlot]);
                turretFound = true;
                break;
            }
        }

        if (!turretFound) {
            robotContainer.gamepadEx2.rumble();
        }
    }

    public double getError() {
        return HelperFunctions.normalizeAngle(targetAngle - robotContainer.spindexer.getAngle());
    }

    public void setPos(double pos) {
        spindexerServo.updateSetPosition(pos);
    }

    public void setPosDegrees(double angle) {
        robotContainer.telemetry.addData("Spindexer Target Angle", angle);
        spindexerServo.updateSetPosition((HelperFunctions.normalizeAngle(angle + Constants.Spindexer.ANGLE_OFFSET) + 180) /360);
        robotContainer.telemetry.addData("Spindexer Angle After Math", (HelperFunctions.normalizeAngle(angle) + 180)/360);
    }
}
