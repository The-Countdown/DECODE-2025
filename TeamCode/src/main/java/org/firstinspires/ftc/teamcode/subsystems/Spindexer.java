package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.BetterColorSensor;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final BetterCRServo spindexerServo;
    private final BetterAnalogInput spindexAnalog;
    private final BetterColorSensor colorSensor;
    public double targetAngle;
    public double lastPosition;
    private double error;
    private double lastError;
    private double p;
    private double d;
    private double ff;
    private boolean pause;
    private boolean direction;
    private ElapsedTime beamTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();

    public Spindexer (RobotContainer robotContainer, BetterCRServo spindexerServo, BetterAnalogInput spindexAnalog, BetterColorSensor colorSensor) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServo;
        this.spindexAnalog = spindexAnalog;
        this.colorSensor = colorSensor;
        this.lastPosition = getRawAngle();
        this.targetAngle = 0;
        this.lastPosition = 0;
        this.pause = false;
        this.beamTimer.reset();
    }

    public void update(boolean teleop) {
        double spindexerError = Math.abs(getError());
        boolean jammed = jammed();
        robotContainer.telemetry.addData("jammed:", jammed);

        if (spindexerError > 5 && !this.pause) {
            spindexerServo.updateSetPower(robotContainer.spindexer.pdf.calculate());
        } else if (!this.pause) {
            spindexerServo.updateSetPower(0);
        }
        //Should this stop running when right bumper is held???? -Elliot

        if (teleop) {
            if (robotContainer.beamBreakToggleButton.wasJustPressed() && beamTimer.seconds() > 0.8) {
                robotContainer.delayedActionManager.schedule(()-> robotContainer.spindexer.moveIntakeSlotClockwise(), 500);
                beamTimer.reset();
            }

            if (robotContainer.gamepadEx1.leftBumper.wasJustPressed()) {
                robotContainer.spindexer.moveIntakeSlotClockwise();
            }

            if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
                spindexerServo.updateSetPower(0);
            }


                if (robotContainer.gamepadEx1.dpadUp.isHeld()) {
                Status.intakeToggle = false;
                Status.turretToggle = true;
                this.pause = true;
            }

            if (robotContainer.gamepadEx1.rightBumper.isHeld()) {
                spindexerServo.updateSetPower(1);
            }

            if (robotContainer.gamepadEx1.dpadUp.wasJustReleased()) {
                Status.intakeToggle = true;
                Status.turretToggle = false;
                spindexerServo.updateSetPower(0);
                this.pause = false;
            }

            // Start up turret
            if (robotContainer.gamepadEx2.square.wasJustPressed()) {
                Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
                Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
                Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
                Status.intakeToggle = false;
                Status.turretToggle = true;
            }

            // Stop turret
            if (robotContainer.gamepadEx2.triangle.wasJustPressed()) {
                Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.NONE;
                Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.NONE;
                Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.NONE;
                Status.intakeToggle = true;
                Status.turretToggle = false;
            }
        }
        this.lastPosition = getAngle();
    }

    public void goToNextIntakeSlot(boolean nothing) {
    }

    public double getAngle() {
        double angle = (spindexAnalog.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;
        angle += Constants.Spindexer.ANGLE_OFFSET;
        angle = angle % 360;
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

    public void function() {
        robotContainer.spindexer.updateSlot();
        robotContainer.spindexer.goToNextIntakeSlot(true);
    }

    public void autoFunction() {
        robotContainer.spindexer.updateSlot();
        if (robotContainer.spindexer.isFull()) { // If it is full after an intake
        } else {
            robotContainer.spindexer.goToNextIntakeSlot(true);
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
        targetAngle = Constants.Spindexer.INTAKE_SLOT_ANGLES[(currentSlot + 1) % 3];
    }

    public void shootNextBall(boolean matchMotif) {
        Status.ballsToShoot -= 1;
        Status.turretToggle = true;
        Status.intakeToggle = false;
        Status.flywheelToggle = true;

        robotContainer.delayedActionManager.schedule(()-> robotContainer.transfer.flapUp(), 1200);
        robotContainer.delayedActionManager.schedule(()-> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME + 1200);
        robotContainer.delayedActionManager.schedule(()-> Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE, Constants.Transfer.FLIP_TIME + 1200);

        if (Status.ballsToShoot > 0) {
            robotContainer.delayedActionManager.schedule(() -> shootNextBall(matchMotif), Constants.Transfer.FLIP_TIME + 1600);
        }
    }

    public void shootAll(boolean matchMotif) {
        Status.ballsToShoot = 3;
        shootNextBall(matchMotif);
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

    public boolean jammed() {
        double error = Math.abs(getError());
        double speed = Math.abs((getAngle() - this.lastPosition) * robotContainer.DELTA_TIME_MS);
        robotContainer.telemetry.addData("jam error:", error);
        robotContainer.telemetry.addData("jam speed:", speed);
        if (error < 15 && speed > 100) {
            jamTimer.reset();
        }
        if (error > 15 && speed < 100 && jamTimer.seconds() > 0.8) {
            return true;
        } else {
            return false;
        }
    }

    public double getError() {
        return getAngle() - (targetAngle);
    }

    // For reset at the start of teleop or something
    public void goToFirstIntakeSlot() {
        targetAngle = Constants.Spindexer.INTAKE_SLOT_ANGLES[0];
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public class PDF {
        public double calculate() {
            error = HelperFunctions.normalizeAngle(getError());
             robotContainer.telemetry.addData("Spin Calc Error:", error);

            p = Constants.Spindexer.KP * error;
            d = (Constants.Spindexer.KD * (lastError - error) * robotContainer.DELTA_TIME_MS);
            ff = Math.signum(error) * Constants.Spindexer.KF;

             robotContainer.telemetry.addData("p", p);
             robotContainer.telemetry.addData("d", d);
             robotContainer.telemetry.addData("ff", ff);

            lastError = error;
            double f = (p - d) + ff;
             robotContainer.telemetry.addData("f", f);
            return f;
        }
    }
    public PDF pdf = new PDF();
}
