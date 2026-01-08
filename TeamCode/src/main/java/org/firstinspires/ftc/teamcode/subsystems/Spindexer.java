package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedServos;
import org.firstinspires.ftc.teamcode.util.PIDF;

import java.util.Arrays;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final LinkedServos spindexerServo;
    private final BetterAnalogInput spindexerAnalog;
    private final BetterColorSensor colorSensor;
    public double targetAngle;
    public double lastPosition;
    private double spindexerError;
    public boolean pause;
    public boolean clockwise;
    private PIDF spindexerPIDF;
    private ElapsedTime beamTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private ElapsedTime unjamTimer = new ElapsedTime();
    public ElapsedTime waitAtStartTimer = new ElapsedTime();
    public Constants.Game.ARTIFACT_COLOR[] slotColor = {Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN};
    public int slotZeroAngle;
    public Spindexer (RobotContainer robotContainer, LinkedServos spindexerServos, BetterAnalogInput spindexerAnalog, BetterColorSensor colorSensor) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServos;
        this.spindexerAnalog = spindexerAnalog;
        this.colorSensor = colorSensor;
        this.lastPosition = getRawAngle();
        this.targetAngle = 0;
        this.lastPosition = 0;
        this.pause = false;
        this.beamTimer.reset();
        this.spindexerError = 0;
        this.slotZeroAngle = 0;

        Arrays.fill(this.slotColor, Constants.Game.ARTIFACT_COLOR.UNKNOWN);
        spindexerPIDF = new PIDF(robotContainer, Constants.Spindexer.KP, Constants.Spindexer.KI, Constants.Spindexer.KD, Constants.Spindexer.KF);
    }

    // TODO: Change to -180 to 180 instead of 0 - 360
    public void update(boolean teleop) {
        if (waitAtStartTimer.milliseconds() < Constants.Turret.FLYWHEEL_SPINUP_MS && !teleop) {
            return;
        }
        spindexerPIDF = spindexerPIDF.updateValues(robotContainer, Constants.Spindexer.KP, Constants.Spindexer.KI, Constants.Spindexer.KD, Constants.Spindexer.KF);

        spindexerError = getError();
        boolean jammed = jammed();

        double servoPower = calculate();
        if (Math.abs(spindexerError) > 5 && !this.pause) {
            if (clockwise && spindexerError < -20) {
                spindexerServo.setPower(-Math.abs(servoPower));
                // spindexerServo.setPower(Math.abs(servoPower));
            } else {
                spindexerServo.setPower(servoPower);
            }
        } else if (!this.pause) {
            spindexerServo.setPower(0);
            clockwise = false;
        }

        if (robotContainer.beamBreakToggleButton.wasJustReleased() || robotContainer.beamBreakToggleButton.isHeldForAtLeast(0.25)) {
            robotContainer.delayedActionManager.schedule(() -> function2(), Constants.Spindexer.TIME_BETWEEN_BEAM_BREAK_AND_COLOR_SENSOR);
            beamTimer.reset();
        }

        if (this.pause) {
            if (jammed) {
                spindexerServo.setPower(-0.3);
                unjamTimer.reset();
                jamTimer.reset();
            } else if (unjamTimer.seconds() > 0.2) {
                spindexerServo.setPower(1);
            }
        }

        if (teleop) {
            if (robotContainer.gamepadEx1.leftBumper.wasJustPressed()) {
                robotContainer.spindexer.moveIntakeSlotClockwise();
            }

            if (robotContainer.gamepadEx2.leftBumper.wasJustPressed()) {
                robotContainer.spindexer.moveIntakeSlotClockwise();
            }

            if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
                Status.intakeToggle = false;
                Status.turretToggle = true;
            }

            if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
                Status.intakeToggle = false;
                Status.turretToggle = true;
            }

            if (robotContainer.gamepadEx2.rightBumper.isHeld()) {
                this.pause = true;
            }

            if (robotContainer.gamepadEx1.rightBumper.isHeld()) {
                this.pause = true;
            }

            if (robotContainer.gamepadEx2.dpadUp.wasJustReleased()) {
                Status.intakeToggle = true;
                Status.turretToggle = false;
                spindexerServo.setPower(0);
                this.pause = false;
            }

            if (robotContainer.gamepadEx1.dpadUp.wasJustReleased()) {
                Status.intakeToggle = true;
                Status.turretToggle = false;
                spindexerServo.setPower(0);
                this.pause = false;
            }

            if (robotContainer.gamepadEx2.rightBumper.wasJustReleased()) {
                spindexerServo.setPower(0);
                this.pause = false;
                robotContainer.spindexer.goToFirstIntakeSlot();
            }

            if (robotContainer.gamepadEx1.rightBumper.wasJustReleased()) {
                spindexerServo.setPower(0);
                this.pause = false;
                robotContainer.spindexer.goToFirstIntakeSlot();
            }
        }
        this.lastPosition = getAngle();
    }

    public double getAngle() {
        double angle = (spindexerAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
        angle += Constants.Spindexer.ANGLE_OFFSET;
        angle = angle % 360;
        return angle;
    }

    public double getRawAngle() {
        return (spindexerAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
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

    public void function2() {
        if (colorSensor.getDistance() < Constants.Spindexer.DIST_TOLERANCE && Math.abs(robotContainer.spindexer.spindexerError) < 20) {
            slotColor[getCurrentIntakeSlot()] = getArtifactColor(colorSensor.updateBlue(), colorSensor.updateGreen());
            robotContainer.spindexer.moveIntakeSlotClockwise();
            robotContainer.gamepadEx1.rumble(100);
            robotContainer.gamepadEx2.rumble(100);
        }
    }

    public void updateSlot() {
    }

    // Assume the Spindexer is always at target
    public int getCurrentIntakeSlot() {
        int current = (int) ((targetAngle % 360) / 120);
        return (current + 2) % 3;
    }

    // Assume the Spindexer is always at target
    public int getCurrentTransferSlot() {
        int current = (int) ((targetAngle % 360) + 60) / 120;
        return current % 3;
    }

    public void moveIntakeSlotClockwise() {
        clockwise = true; // Disable this to remove clockwise functionality
        int currentSlot = getCurrentIntakeSlot();
        targetAngle = (targetAngle + 120) % 360;
    }

    public void shootToggle(boolean shootToggle){
        Status.turretToggle = shootToggle;
        Status.intakeToggle = !shootToggle;
        Status.flywheelToggle = shootToggle;
    }
    public void shootAll(boolean matchMotif) {
        shootToggle(true);
        if (matchMotif) {
            shootAll(Status.motif);
        } else {
            pause();
            robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(),  Constants.Spindexer.FULL_EMPTY_SPINTIME);
        }
    }
    public void shootAll(Constants.Game.MOTIF motif){
        if (motif == Constants.Game.MOTIF.PPG){
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 0);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 750);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 1500);
        } else if (motif == Constants.Game.MOTIF.PGP){
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 0);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 750);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 1500);
        } else if (motif == Constants.Game.MOTIF.GPP){
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 0);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 750);
            robotContainer.delayedActionManager.schedule( ()-> goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR.PURPLE), 1500);
        }
    }

    public void pause() {
        this.pause = true;
    }

    public void unpause() {
        this.pause = false;
    }

    public boolean isFull() {
        int balls = 0;
        for (int i = 0; i < Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
            if (robotContainer.spindexer.slotColor[i] == Constants.Game.ARTIFACT_COLOR.PURPLE || robotContainer.spindexer.slotColor[i] == Constants.Game.ARTIFACT_COLOR.GREEN) {
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
            if (robotContainer.spindexer.slotColor[i] == Constants.Game.ARTIFACT_COLOR.PURPLE || robotContainer.spindexer.slotColor[i] == Constants.Game.ARTIFACT_COLOR.GREEN) {

                balls += 1;
            }
        }
        if (balls > 0) {
            return false;
        } else {
            return true;
        }
    }

    public void goToNextColorIntakeSlot(Constants.Game.ARTIFACT_COLOR color){
       for (int i = 0; i < Constants.Spindexer.INTAKE_SLOT_ANGLES.length; i++) {
           if (slotColor[i] == color){
               for (int j = ((i*120) + slotZeroAngle)/120;  j >0; j--){
                   moveIntakeSlotClockwise();
               }
               moveIntakeCounterClockwise();
           }
       }
    }

    public void shootAllMotifOrder(boolean onlyMotif) {
        Status.turretToggle = true;
        Status.intakeToggle = false;
        Status.flywheelToggle = true;

        int closestSlot = (getCurrentTransferSlot() + 1) % 3;
        int secondClosestSlot = getCurrentTransferSlot();
        int thirdClosestSlot = (getCurrentTransferSlot() - 1 + 3) % 3;

        int[] slots = {closestSlot, secondClosestSlot, thirdClosestSlot};
        Constants.Game.ARTIFACT_COLOR[] slotColors = {
                slotColor[closestSlot],
                slotColor[secondClosestSlot],
                slotColor[thirdClosestSlot]
        };

        Constants.Game.ARTIFACT_COLOR[] motifColors = getMotifColors(Status.motif);

        int firstSlot = pickFirstSlot(slots, slotColors, motifColors, onlyMotif);
        if (firstSlot == -1) return;
        int secondSlot = pickSecondSlot(firstSlot, slots, slotColors, motifColors, onlyMotif);
        if (secondSlot == -1) return;
        int thirdSlot = pickThirdSlot(firstSlot, secondSlot, slots, slotColors, motifColors, onlyMotif);
        if (thirdSlot == -1) return;

        shootSlots(firstSlot, secondSlot, thirdSlot);
    }


    private int pickFirstSlot(int[] slots, Constants.Game.ARTIFACT_COLOR[] slotColors, Constants.Game.ARTIFACT_COLOR[] motifColors, boolean onlyMotif) {
        Constants.Game.ARTIFACT_COLOR firstColor = motifColors[0];

        if (Status.motif == Constants.Game.MOTIF.UNKNOWN || (slotColors[0] != firstColor && slotColors[1] != firstColor && slotColors[2] != firstColor)) {
            if (!onlyMotif) {
                robotContainer.delayedActionManager.schedule(() -> this.pause = true, () -> robotContainer.turret.flywheel.atTargetVelocity());
                robotContainer.delayedActionManager.schedule(() -> this.pause = false, () -> robotContainer.turret.flywheel.atTargetVelocity(), Constants.Spindexer.FULL_EMPTY_SPINTIME);
            }
            return -1;
        }

        for (int i = 0; i < 3; i++) {
            if (slotColors[i] == firstColor) {
                return slots[i];
            }
        }

        return -1;
    }

    private int pickSecondSlot(int firstSlot, int[] slots, Constants.Game.ARTIFACT_COLOR[] slotColors, Constants.Game.ARTIFACT_COLOR[] motifColors, boolean onlyMotif) {
        Constants.Game.ARTIFACT_COLOR secondColor = motifColors[1];

        for (int i = 0; i < 3; i++) {
            if (slots[i] != firstSlot && slotColors[i] == secondColor) {
                return slots[i];
            }
        }

        if (onlyMotif) {
            shootSlot(firstSlot);
            return -1;
        }

        for (int i = 0; i < 3; i++) {
            if (slots[i] != firstSlot) return slots[i];
        }

        return slots[0];
    }


    private int pickThirdSlot(int firstSlot, int secondSlot, int[] slots, Constants.Game.ARTIFACT_COLOR[] slotColors, Constants.Game.ARTIFACT_COLOR[] motifColors, boolean onlyMotif) {
        Constants.Game.ARTIFACT_COLOR thirdColor = motifColors[2];

        for (int i = 0; i < 3; i++) {
            if (slots[i] != firstSlot && slots[i] != secondSlot && slotColors[i] == thirdColor) {
                return slots[i];
            }
        }

        if (onlyMotif) {
            shootSlots(firstSlot, secondSlot);
            return -1;
        }

        for (int i = 0; i < 3; i++) {
            if (slots[i] != firstSlot && slots[i] != secondSlot) return slots[i];
        }

        return slots[0];
    }


    public void shootSlots(int firstSlotIndex, int secondSlotIndex, int thirdSlotIndex) {
        if ((firstSlotIndex + 1) % 3 == secondSlotIndex && (secondSlotIndex + 1) % 3 == thirdSlotIndex) {
            setTargetAngle(Constants.Spindexer.BEFORE_TRANSFER_SLOT_ANGLES[firstSlotIndex]);
            robotContainer.delayedActionManager.schedule(() -> setTargetAngle(Constants.Spindexer.AFTER_TRANSFER_SLOT_ANGLES[thirdSlotIndex]), () -> Math.abs(getError()) < 5);
        } else {
            shootSlot(firstSlotIndex);
            shootSlot(secondSlotIndex);
            shootSlot(thirdSlotIndex);
        }
    }

    public void shootSlots(int firstSlotIndex, int secondSlotIndex) {
        if ((firstSlotIndex + 1) % 3 == secondSlotIndex) {
            setTargetAngle(Constants.Spindexer.BEFORE_TRANSFER_SLOT_ANGLES[firstSlotIndex]);
            robotContainer.delayedActionManager.schedule(() -> setTargetAngle(Constants.Spindexer.AFTER_TRANSFER_SLOT_ANGLES[secondSlotIndex]), () -> Math.abs(getError()) < 5);
        } else {
            shootSlot(firstSlotIndex);
            shootSlot(secondSlotIndex);
        }
    }

    public void shootSlot(int slotIndex) {
        setTargetAngle(Constants.Spindexer.BEFORE_TRANSFER_SLOT_ANGLES[slotIndex]);
        robotContainer.delayedActionManager.schedule(() -> setTargetAngle(Constants.Spindexer.AFTER_TRANSFER_SLOT_ANGLES[slotIndex]), () -> Math.abs(getError()) < 5);
    }

    private Constants.Game.ARTIFACT_COLOR[] getMotifColors(Constants.Game.MOTIF motif) {
        switch (motif) {
            case GPP: return new Constants.Game.ARTIFACT_COLOR[]{Constants.Game.ARTIFACT_COLOR.GREEN, Constants.Game.ARTIFACT_COLOR.PURPLE, Constants.Game.ARTIFACT_COLOR.PURPLE};
            case PGP: return new Constants.Game.ARTIFACT_COLOR[]{Constants.Game.ARTIFACT_COLOR.PURPLE, Constants.Game.ARTIFACT_COLOR.GREEN, Constants.Game.ARTIFACT_COLOR.PURPLE};
            case PPG: return new Constants.Game.ARTIFACT_COLOR[]{Constants.Game.ARTIFACT_COLOR.PURPLE, Constants.Game.ARTIFACT_COLOR.PURPLE, Constants.Game.ARTIFACT_COLOR.GREEN};
            default:  return new Constants.Game.ARTIFACT_COLOR[]{Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN};
        }
    }


    public void moveIntakeCounterClockwise(){
        targetAngle = (targetAngle - 120) % 360;
    }

    public boolean jammed() {
        double error = Math.abs(getError());
        double speed = Math.abs((getAngle() - this.lastPosition) * robotContainer.CURRENT_LOOP_TIME_MS);
        if (error < 15 && speed > 10) {
            jamTimer.reset();
        }
        if (error > 15 && speed < 10 && jamTimer.seconds() > Constants.Spindexer.JAM_TIME_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public double getError() {
        return HelperFunctions.normalizeAngle(getAngle() - (targetAngle));
    }

    // For reset at the start of teleop or something
    public void goToFirstIntakeSlot() {
        targetAngle = Constants.Spindexer.INTAKE_SLOT_ANGLES[0];
        slotZeroAngle = (int) Math.round((getAngle() / 120.0) * 120);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double calculate() {
        return spindexerPIDF.update(getError(), robotContainer.CURRENT_LOOP_TIME_MS);
//            error = HelperFunctions.normalizeAngle(getError());
//            robotContainer.telemetry.addData("Spin Calc Error:", error);
//
//            p = Constants.Spindexer.KP * error;
//            ff = Math.signum(error) * Constants.Spindexer.KF;
//
//            robotContainer.telemetry.addData("p", p);
//            robotContainer.telemetry.addData("d", d);
//            robotContainer.telemetry.addData("ff", ff);
//
//            lastError = error;
//            double f = (p - d) + ff;
//            robotContainer.telemetry.addData("f", f);
//            if (error > 80) {
//                return f * -1;
//            } else {
//                return f;
//            }
    }
}
