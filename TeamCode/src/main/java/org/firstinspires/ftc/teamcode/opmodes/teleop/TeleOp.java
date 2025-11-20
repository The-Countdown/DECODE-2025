package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.main.Constants.Spindexer.axonTestAngle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public double turretPos = 0;
    private final GamepadWrapper.ButtonReader transferConditionButton = new GamepadWrapper.ButtonReader();
    private final ElapsedTime spinTimer = new ElapsedTime();


    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        // Get the blackboard pose that was set during auto or if it was not set set the starting pose to 0.
        RobotContainer.HardwareDevices.pinpoint.setPosition((Pose2D) blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.intakeToggle = true;
        Status.turretToggle = false;
        Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        RobotContainer.HardwareDevices.limelight.start();
        robotContainer.start(this, false);
        Status.isDrivingActive = true;
        robotContainer.spindexer.setPosDegrees(Constants.Spindexer.INTAKE_SLOT_ANGLES[0]);
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();
        robotContainer.limelightLogic.update();
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus();
        Status.turretToggleButton.update(Status.turretToggle);

        robotContainer.allIndicatorLights.lightsUpdate();

        // Get current and voltage for telemetry
        robotContainer.controlHubVoltage = robotContainer.getVoltage(Constants.Robot.CONTROL_HUB_INDEX);
        robotContainer.expansionHubVoltage = robotContainer.getVoltage(Constants.Robot.EXPANSION_HUB_INDEX);
        robotContainer.controlHubCurrent = robotContainer.getCurrent(Constants.Robot.CONTROL_HUB_INDEX);
        robotContainer.expansionHubCurrent = robotContainer.getCurrent(Constants.Robot.EXPANSION_HUB_INDEX);

        // Gamepad 1
        robotContainer.drivetrain.controlUpdate();
        robotContainer.turret.hood.setPos(robotContainer.gamepadEx1.circle.isPressed() ? Constants.Turret.HOOD_PRESETS[1] : Constants.Turret.HOOD_PRESETS[0]);

        if (robotContainer.gamepadEx1.options.wasJustPressed()) {
            Status.manualControl = !Status.manualControl;
        }

        if (robotContainer.gamepadEx1.dpadLeft.wasJustPressed()) {
            robotContainer.spindexer.alwaysGoBackOneIntakeSlot();
        }

        if (robotContainer.gamepadEx1.dpadRight.wasJustPressed()) {
            robotContainer.spindexer.alwaysGoToNextIntakeSlot();
        }

        if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
            robotContainer.spindexer.slotUpdate(); // Fill the current slot
        }

        // Gamepad 2

        // Intake - Circle
        Status.intakeToggle = robotContainer.gamepadEx2.circle.wasJustPressed() != Status.intakeToggle;
        if (Status.intakeToggle) {
            robotContainer.intake.setPower(Status.intakeToggle ? (robotContainer.gamepadEx1.rightTriggerRaw() - robotContainer.gamepadEx1.leftTriggerRaw()) * Constants.Intake.TOP_SPEED : 0);
        } else {
            robotContainer.intake.setPower(Constants.Intake.REVERSE_TOP_SPEED);
        }

        // Flywheel - Automated
        if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
            Status.turretToggle = true;
            robotContainer.spindexer.goToNextTransferSlot();
        } else if (robotContainer.gamepadEx2.circle.wasJustReleased() && Status.intakeToggle) {
            Status.turretToggle = false;
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        if (Status.manualControl && robotContainer.gamepadEx2.dpadRight.isHeld()) {
            robotContainer.turret.flywheel.setTargetVelocity(Math.min(Math.pow(robotContainer.gamepadEx2.dpadRight.getHoldDuration(), Constants.Turret.FLYWHEEL_CURVE), 1));
        } else if (!Status.intakeToggle) {
            robotContainer.turret.flywheel.setTargetVelocity(Math.min(Status.turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal())));
        } else {
            robotContainer.turret.flywheel.setTargetVelocity(0);
        }

        Status.flywheelAtTargetSpeed = robotContainer.turret.flywheel.atTargetVelocity();

        // Turret turn - Right stick X
        if (Status.manualControl) {
            turretPos -= robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
            turretPos = HelperFunctions.clamp(turretPos, Constants.Turret.TURRET_LIMIT_MIN, Constants.Turret.TURRET_LIMIT_MAX);
            robotContainer.turret.setTargetPosition(turretPos);
        } else {
            robotContainer.turret.pointAtGoal();
        }

        // For when automatic spindexer fails
        if (robotContainer.gamepadEx2.square.wasJustPressed()) {
            Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
            Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
            Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
            Status.intakeToggle = false;
            Status.turretToggle = true;
            robotContainer.spindexer.goToNextTransferSlot();
        }

        if (robotContainer.gamepadEx2.triangle.wasJustPressed()) {
            Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.NONE;
            Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.NONE;
            Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.NONE;
            Status.intakeToggle = true;
            Status.turretToggle = false;
            robotContainer.spindexer.alwaysGoToNextIntakeSlot();
        }

        if (robotContainer.gamepadEx2.cross.wasJustPressed()) {
            Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE;
            robotContainer.spindexer.goToNextTransferSlot();
            if (robotContainer.spindexer.isEmpty()) {
                Status.intakeToggle = true;
                Status.turretToggle = false;
                robotContainer.spindexer.goToNextIntakeSlot();
            }
        }

        if (robotContainer.gamepadEx2.leftBumper.wasJustPressed()) {
            robotContainer.transfer.flapUp();
        } else if (robotContainer.gamepadEx2.leftBumper.wasJustReleased()) {
            robotContainer.transfer.flapDown();
        }

        if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
            robotContainer.spindexer.goToNextGreenSlot();
        }
        if (robotContainer.gamepadEx2.dpadDown.wasJustPressed()) {
            robotContainer.spindexer.goToNextPurpleSlot();
        }

        // No gamepad

        // Update the breamBreak state
        robotContainer.beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());

        if (robotContainer.beamBreakToggleButton.wasJustReleased() && robotContainer.intake.getPower() > 0 && spinTimer.milliseconds() > 200) {
            robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.function(), Constants.Spindexer.COLOR_SENSE_TIME);
            spinTimer.reset();
        }

        // If all are none or unknown, turret toggle
        if ((Status.slotColor[0] != Constants.Game.ARTIFACT_COLOR.NONE && Status.slotColor[1] != Constants.Game.ARTIFACT_COLOR.NONE && Status.slotColor[2] != Constants.Game.ARTIFACT_COLOR.NONE) || (Status.slotColor[0] != Constants.Game.ARTIFACT_COLOR.UNKNOWN && Status.slotColor[1] != Constants.Game.ARTIFACT_COLOR.UNKNOWN && Status.slotColor[2] != Constants.Game.ARTIFACT_COLOR.UNKNOWN)) {
            Status.turretToggle = true;
        }

        if (robotContainer.limelightLogic.limelight.getLatestResult().isValid()) {
            robotContainer.telemetry.addData("robot pos on field", robotContainer.limelightLogic.logicBotPose());
            robotContainer.telemetry.addData("distance to goal", robotContainer.limelightLogic.disToGoal());
        } else if (robotContainer.limelightLogic.limelight.getLatestResult() == null){
            robotContainer.telemetry.addLine("robot pos on field no see");
        }

        robotContainer.telemetry.addData("turret interpolation", robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));

        robotContainer.telemetry("teleOp");
        Thread.yield();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);

        Status.isDrivingActive = false;
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
