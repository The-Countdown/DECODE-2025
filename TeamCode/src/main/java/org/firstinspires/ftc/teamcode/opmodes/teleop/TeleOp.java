package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public double turretPos = 0;

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

        robotContainer.drivetrain.controlUpdate();

        robotContainer.allIndicatorLights.lightsUpdate();

        turretPos += robotContainer.gamepadEx1.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx1.rightStickX(), 3) : 0;
        robotContainer.turret.setTargetPosition(turretPos);

//        robotContainer.turret.flywheel.setTargetVelocity(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));

        Status.intakeEnabled = robotContainer.gamepadEx1.circle.wasJustPressed() != Status.intakeEnabled;
        robotContainer.intake.setIntakeVelocity(Status.intakeEnabled ? 1 : 0);

        if (robotContainer.gamepadEx2.dpadUp.isPressed()) {
            robotContainer.spindexer.setTargetAngle(Constants.Spindexer.INTAKE_SLOT_ANGLES[0]);
        } else if (robotContainer.gamepadEx2.dpadDown.isPressed()) {
            robotContainer.spindexer.setTargetAngle(Constants.Spindexer.INTAKE_SLOT_ANGLES[1]);
        } else if (robotContainer.gamepadEx2.dpadRight.isPressed()) {
            robotContainer.spindexer.setTargetAngle(Constants.Spindexer.INTAKE_SLOT_ANGLES[2]);
        }
        RobotContainer.HardwareDevices.spindexServo.setPower(robotContainer.spindexer.pidf.calculate());

        robotContainer.spindexer.slotsUpdate();

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
