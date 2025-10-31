package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
    private final GamepadWrapper.ButtonReader turretToggleButton = new GamepadWrapper.ButtonReader();
    private final GamepadWrapper.ButtonReader transferConditionButton = new GamepadWrapper.ButtonReader();
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;
    private double lastTransfer = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.setPosition((Pose2D) blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));
        robotContainer.telemetry.addData("hello: ", blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        // robotContainer.refreshData();
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;
        Status.intakeToggle = true;
        Status.turretToggle = false;
        Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        RobotContainer.HardwareDevices.limelight.start();
        robotContainer.start(this);
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();
        robotContainer.limelightLogic.update();
        turretToggleButton.update(Status.turretToggle);
        robotContainer.beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());

        robotContainer.allIndicatorLights.lightsUpdate();

        //gamepad 1

        robotContainer.drivetrain.controlUpdate();

        robotContainer.turret.hood.setPos(robotContainer.gamepadEx1.circle.isPressed() ? Constants.Turret.HOOD_PRESETS[1] : Constants.Turret.HOOD_PRESETS[0]);

        //gamepad 2

        //intake -circle
        Status.intakeToggle = robotContainer.gamepadEx2.circle.wasJustPressed() != Status.intakeToggle;
        robotContainer.intake.setVelocity(Status.intakeToggle ? (robotContainer.gamepadEx1.rightTriggerRaw() - robotContainer.gamepadEx1.leftTriggerRaw()) * 0.8 : 0);

        //flywheel -automated
        if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
            Status.turretToggle = true;
            robotContainer.spindexer.goToNextTransferSlot();
        } else if (robotContainer.gamepadEx2.circle.wasJustReleased() && Status.intakeToggle) {
            Status.turretToggle = false;
        }

        if (Status.manualControl) {
            robotContainer.turret.flywheel.setTargetVelocity(turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE);
        } else if (!Status.intakeToggle) {
            robotContainer.turret.flywheel.setTargetVelocity(Math.min(turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.limelightLogic.useInterpolate()));
        } else {
            robotContainer.turret.flywheel.setTargetVelocity(0);
        }

//        turret turn -right stick X
        if (Status.manualControl) {
            turretPos -= robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
            turretPos = HelperFunctions.clamp(turretPos, Constants.Turret.TURRET_LIMIT_MIN, Constants.Turret.TURRET_LIMIT_MAX);
            robotContainer.turret.setTargetPosition(turretPos);
        } else {
            robotContainer.turret.pointAtGoal();
        }
//
//        if (robotContainer.limelightLogic.limelight.getLatestResult().isValid()) {
//            robotContainer.limelightLogic.trackGoal();
//        } else {
//            robotContainer.turret.pointAtGoal();
//        }

        if (robotContainer.beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        double spindexerError = Math.abs(robotContainer.spindexer.pidf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - spindexerError) > 50) {
            spindexAccel.reset();
        }

        if (spindexerError > 1) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pidf.calculate() * spindexAccel.seconds(), 0.5));
            } else {
                 robotContainer.spindexer.setPower(robotContainer.spindexer.pidf.calculate());
//                robotContainer.spindexer.setPower(0.5);
            }
        } else {
            robotContainer.spindexer.setPower(0);
        }
        lastError = spindexerError;

        if (robotContainer.gamepadEx1.options.wasJustPressed()) {
            Status.manualControl = !Status.manualControl;
        }

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
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        //if all are none or unknown, turret toggle
        if ((Status.slotColor[0] != Constants.Game.ARTIFACT_COLOR.NONE && Status.slotColor[1] != Constants.Game.ARTIFACT_COLOR.NONE && Status.slotColor[2] != Constants.Game.ARTIFACT_COLOR.NONE) || (Status.slotColor[0] != Constants.Game.ARTIFACT_COLOR.UNKNOWN && Status.slotColor[1] != Constants.Game.ARTIFACT_COLOR.UNKNOWN && Status.slotColor[2] != Constants.Game.ARTIFACT_COLOR.UNKNOWN)) {
            Status.turretToggle = true;
        }

        if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
            Pose2D botPose = robotContainer.limelightLogic.logicBotPose();
            RobotContainer.HardwareDevices.pinpoint.setPosX(botPose.getX(DistanceUnit.CM), DistanceUnit.CM);
            RobotContainer.HardwareDevices.pinpoint.setPosY(botPose.getY(DistanceUnit.CM), DistanceUnit.CM);
        }

        if (robotContainer.gamepadEx2.cross.wasJustPressed()) {
            Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE;
            robotContainer.spindexer.goToNextTransferSlot();
        }

        if (robotContainer.gamepadEx2.leftBumper.isPressed()) {
            robotContainer.transfer.flapUp();
        } else if (robotContainer.gamepadEx2.rightBumper.isPressed()) {
            robotContainer.transfer.flapDown();
        }

        transferConditionButton.update((robotContainer.spindexer.getTargetAngle() == Constants.Spindexer.TRANSFER_SLOT_ANGLES[0] || robotContainer.spindexer.getTargetAngle() == Constants.Spindexer.TRANSFER_SLOT_ANGLES[1] || robotContainer.spindexer.getTargetAngle() == Constants.Spindexer.TRANSFER_SLOT_ANGLES[2]) && spindexerError < 6 && robotContainer.turret.flywheel.atTargetVelocity() && !Status.intakeToggle);
        if (transferConditionButton.wasJustPressed() && lastTransfer != robotContainer.spindexer.getTargetAngle() && robotContainer.turret.atTarget()) {
            robotContainer.transfer.flapUp();
            robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME);
            robotContainer.delayedActionManager.schedule(() -> lastTransfer = robotContainer.spindexer.getTargetAngle(), Constants.Transfer.FLIP_TIME * 5);
        }

        //TODO DA HOOD

        robotContainer.telemetry.addData("hood angle", robotContainer.turret.hoodServo.getPosition());
        if (robotContainer.limelightLogic.limelight.getLatestResult().isValid()) {
            robotContainer.telemetry.addData("robot pos on field", robotContainer.limelightLogic.logicBotPose());
            robotContainer.telemetry.addData("disTANCE to goal", robotContainer.limelightLogic.disToGoal());
        } else if (robotContainer.limelightLogic.limelight.getLatestResult() == null){
            robotContainer.telemetry.addLine("robot pos on field no see");
        }

        robotContainer.telemetry.addData("turret interpolation", robotContainer.limelightLogic.useInterpolate());

        if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
            robotContainer.spindexer.goToNextGreenSlot();
        }
        if (robotContainer.gamepadEx2.dpadDown.wasJustPressed()) {
            robotContainer.spindexer.goToNextPurpleSlot();
        }

//        robotContainer.limelightLogic.trackGoal();

        robotContainer.telemetry("teleOp");

        Thread.yield();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);

        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
