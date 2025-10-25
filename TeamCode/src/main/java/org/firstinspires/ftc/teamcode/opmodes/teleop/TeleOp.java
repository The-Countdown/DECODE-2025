package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public double turretPos = 0;
    private final GamepadWrapper.ButtonReader turretToggleButton = new GamepadWrapper.ButtonReader();
    private final GamepadWrapper.ButtonReader beamBreakToggleButton = new GamepadWrapper.ButtonReader();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
        robotContainer.telemetry.addLine("OpMode Initialized");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;
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
        beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());

        robotContainer.allIndicatorLights.lightsUpdate();

        //gamepad 1

        robotContainer.drivetrain.controlUpdate();

        //intake -circle
        Status.intakeToggle = robotContainer.gamepadEx2.circle.wasJustPressed() != Status.intakeToggle;
        robotContainer.intake.setVelocity(Status.intakeToggle ? 1 : 0);

        //gamepad 2

        //flywheel -automated
        if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
            Status.turretToggle = true;
        }

        if (turretToggleButton.wasJustPressed()) {
            robotContainer.spindexer.goToNextTransferSlot();
        }
        if (Status.turretToggle) {
            robotContainer.turret.flywheel.setTargetVelocity(Math.min(turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));
        }

        //turret turn -right stick X
        turretPos += robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
        robotContainer.turret.setTargetPosition(turretPos);

        //turret flywheels -cross
        robotContainer.turret.flywheel.setTargetVelocity(Math.min(robotContainer.gamepadEx2.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));

        if (beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }
        robotContainer.spindexer.setPower(robotContainer.spindexer.pidf.calculate());

        //transfer -right bumper
        if (robotContainer.gamepadEx2.rightBumper.wasJustPressed()) {
            robotContainer.transfer.setHighPower(1);
            robotContainer.transfer.flapUp();
            robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME);
        } else if (robotContainer.gamepadEx2.rightBumper.wasJustReleased()) {
            robotContainer.transfer.setHighPower(0);
        }

        //manual transfer top wheel -right trigger
        robotContainer.transfer.setHighPower(robotContainer.gamepadEx2.rightTriggerRaw());

        if (robotContainer.gamepadEx2.cross.wasJustPressed()) {
            Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE;
            robotContainer.spindexer.goToNextTransferSlot();
        }

        //hood preset -triangle
        if (robotContainer.gamepadEx2.triangle.wasJustPressed()) {
            robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
        }

        if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
            robotContainer.spindexer.goToNextGreenSlot();
        }
        if (robotContainer.gamepadEx2.dpadDown.wasJustPressed()) {
            robotContainer.spindexer.goToNextPurpleSlot();
        }

        robotContainer.turret.pointAtGoal();

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
