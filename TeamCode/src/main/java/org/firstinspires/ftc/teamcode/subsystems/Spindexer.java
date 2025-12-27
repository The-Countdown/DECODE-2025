package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;
import org.firstinspires.ftc.teamcode.util.LinkedServos;
import org.firstinspires.ftc.teamcode.util.PIDF;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final LinkedServos spindexerServo;
    private final BetterAnalogInput spindexerAnalog;
    private final BetterColorSensor colorSensor;
    public double targetAngle;
    public double lastPosition;
    private double error;
    private double spindexerError;
    private double lastError;
    private double p;
    private double d;
    private double ff;
    public boolean pause;
    private double lastP;
    private boolean direction;
    private PIDF spindexerPIDF;
    private ElapsedTime beamTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private ElapsedTime unjamTimer = new ElapsedTime();
    public Constants.Game.ARTIFACT_COLOR[] slotColor = {Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN, Constants.Game.ARTIFACT_COLOR.UNKNOWN};

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
        for (int i = 0; i < this.slotColor.length; i++) {
            this.slotColor[i] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        }
        spindexerPIDF = new PIDF(robotContainer, Constants.Spindexer.KP, Constants.Spindexer.KD, Constants.Spindexer.KI, Constants.Spindexer.KF);
    }

    public void update(boolean teleop) {
        spindexerError = Math.abs(getError());
        boolean jammed = jammed();
        robotContainer.telemetry.addData("jammed:", jammed);

        robotContainer.telemetry.addData("Spin Error:", calculate());
        if (spindexerError > 5 && !this.pause) {
            spindexerServo.setPower(calculate());
        } else if (!this.pause) {
            spindexerServo.setPower(0);
            spindexerPIDF.reset();
        }

        if (Constants.Spindexer.KP != lastP) {
            spindexerPIDF = new PIDF(robotContainer, Constants.Spindexer.KP, Constants.Spindexer.KD, Constants.Spindexer.KI, Constants.Spindexer.KF);
            lastP = Constants.Spindexer.KP;
        }


        if (robotContainer.beamBreakToggleButton.wasJustReleased() || robotContainer.beamBreakToggleButton.isHeldFor(0.1)) {
            robotContainer.delayedActionManager.schedule(() -> function2(), Constants.Spindexer.COLOR_SENSE_TIME);
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
        //Should this stop running when right bumper is held???? -Elliot

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
        if (colorSensor.getDistance() < Constants.Spindexer.DIST_TOLERANCE && robotContainer.spindexer.spindexerError < 20) {
            robotContainer.spindexer.moveIntakeSlotClockwise();
        } else {
            robotContainer.telemetry.addLine("No ball in distance");
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
        int currentSlot = getCurrentIntakeSlot();
        targetAngle = (targetAngle + 120) % 360;
    }

    public void shootAll(boolean matchMotif) {
        Status.turretToggle = true;
        Status.intakeToggle = false;
        Status.flywheelToggle = true;
        robotContainer.delayedActionManager.schedule(() -> spindexerServo.setPower(1), 0);
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

    public boolean jammed() {
        double error = Math.abs(getError());
        double speed = Math.abs((getAngle() - this.lastPosition) * robotContainer.DELTA_TIME_MS);
        robotContainer.telemetry.addData("jam error:", error);
        robotContainer.telemetry.addData("jam speed:", speed);
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
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double calculate() {
        robotContainer.telemetry.addData("dfs", spindexerPIDF.getPIDFError(targetAngle, robotContainer.DELTA_TIME_MS, getAngle()));
        return spindexerPIDF.update(getError(), robotContainer.DELTA_TIME_MS);
//            error = HelperFunctions.normalizeAngle(getError());
//            robotContainer.telemetry.addData("Spin Calc Error:", error);
//
//            p = Constants.Spindexer.KP * error;
//            d = (Constants.Spindexer.KD * (lastError - error) * robotContainer.DELTA_TIME_MS);
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
