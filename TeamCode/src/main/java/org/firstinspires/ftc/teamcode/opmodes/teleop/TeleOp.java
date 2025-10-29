package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final GamepadWrapper.ButtonReader beamBreakToggleButton = new GamepadWrapper.ButtonReader();
    private final ElapsedTime spindexAccel = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
        robotContainer.spindexer.setTargetAngle(Constants.Spindexer.INTAKE_SLOT_ANGLES[0]);
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
        Status.intakeToggle = false;
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
        beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());

        robotContainer.allIndicatorLights.lightsUpdate();

        //gamepad 1

        robotContainer.drivetrain.controlUpdate();

        //intake -circle
        Status.intakeToggle = robotContainer.gamepadEx2.circle.wasJustPressed() != Status.intakeToggle;
        robotContainer.intake.setVelocity(Status.intakeToggle ? robotContainer.gamepadEx1.rightTriggerRaw() - robotContainer.gamepadEx1.leftTriggerRaw() : 0);

        //gamepad 2

        //flywheel -automated
        if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
            Status.turretToggle = true;
        } else if (robotContainer.gamepadEx2.circle.wasJustReleased() && Status.intakeToggle) {
            Status.turretToggle = false;
        }

        robotContainer.turret.hood.setPos(robotContainer.gamepadEx1.cross.isPressed() ? Constants.Turret.HOOD_PRESETS[1] : Constants.Turret.HOOD_PRESETS[0]);


        if (turretToggleButton.wasJustPressed()) {
            robotContainer.spindexer.goToNextTransferSlot();
        }
        robotContainer.turret.flywheel.setTargetVelocity(Math.min(turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));

//        turret turn -right stick X
        turretPos += robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
        turretPos = HelperFunctions.clamp(turretPos, Constants.Turret.TURRET_LIMIT_MIN, Constants.Turret.TURRET_LIMIT_MAX);
        robotContainer.turret.setTargetPosition(turretPos);

        //turret flywheels -cross
//        robotContainer.turret.flywheel.setTargetVelocity(Math.min(robotContainer.gamepadEx2.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));

        if (robotContainer.gamepadEx1.dpadLeft.wasJustPressed()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        } else if (robotContainer.gamepadEx1.dpadRight.wasJustPressed()) {
            robotContainer.spindexer.goToNextTransferSlot();
        }

        if (beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        double error = Math.abs(robotContainer.spindexer.pidf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - error) > 50) {
            spindexAccel.reset();
            robotContainer.telemetry.addData("Reset Timer", "Spindexer");
        }

        robotContainer.telemetry.addData("Error", error);    
        robotContainer.telemetry.addData("Last error", lastError);    
        robotContainer.telemetry.addData("Diff error", Math.abs(lastError - error));    
        if (error > 4) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pidf.calculate() * spindexAccel.seconds(), 0.5));
            } else {
                 robotContainer.spindexer.setPower(robotContainer.spindexer.pidf.calculate());
//                robotContainer.spindexer.setPower(0.5);
            }
        } else {
            spindexAccel.reset();
            robotContainer.spindexer.setPower(0);
        }
        lastError = error;

//        robotContainer.spindexer.setPower(robotContainer.gamepadEx2.leftTriggerRaw() - robotContainer.gamepadEx2.rightTriggerRaw());

        //transfer -right bumper
//        if (robotContainer.gamepadEx2.leftBumper.wasJustPressed()) {
//            robotContainer.transfer.setHighPower(1);
//            robotContainer.transfer.flapUp();
//            robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME);
//        } else if (robotContainer.gamepadEx2.leftBumper.wasJustReleased()) {
//            robotContainer.transfer.setHighPower(0);
//        }

//        robotContainer.spindexer.setPower(robotContainer.gamepadEx1.leftTriggerRaw() - robotContainer.gamepadEx1.rightTriggerRaw());

        //manual transfer top wheel -right trigger
//        robotContainer.transfer.setHighPower(robotContainer.gamepadEx2.square.isPressed() ? -1 : 0);

        if (robotContainer.gamepadEx2.cross.wasJustPressed()) {
            Status.slotColor[robotContainer.spindexer.getCurrentTransferSlot()] = Constants.Game.ARTIFACT_COLOR.NONE;
            robotContainer.spindexer.goToNextTransferSlot();
        }

        if (robotContainer.gamepadEx2.leftBumper.wasJustPressed()) {
            robotContainer.transfer.flapUp();
        } else if (robotContainer.gamepadEx2.rightBumper.wasJustPressed()) {
            robotContainer.transfer.flapDown();
        }

        //TODO DA HOOD

        robotContainer.telemetry.addData("hood angle", robotContainer.turret.hoodServo.getPosition());
        if (robotContainer.limelightLogic.limelight.getLatestResult().isValid()) {
            robotContainer.telemetry.addData("robot pos on field", robotContainer.limelightLogic.getLimelightPos());
        } else {
            robotContainer.telemetry.addData("robot pos on field", robotContainer.limelightLogic.getLimelightPos());
        }

        //hood preset -triangle
//        if (robotContainer.gamepadEx2.triangle.wasJustPressed()) {
//            robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
//        }
//
//        if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
//            robotContainer.spindexer.goToNextGreenSlot();
//        }
//        if (robotContainer.gamepadEx2.dpadDown.wasJustPressed()) {
//            robotContainer.spindexer.goToNextPurpleSlot();
//        }

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
