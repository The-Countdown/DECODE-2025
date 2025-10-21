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

        //turret speed factor * current loop time is how far u want it to move per how many millisecond(loop time)
        if (robotContainer.gamepadEx1.rightStickY() > 0.1) {
            turretPos += (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx1.rightStickY(), 2);
        } else if (robotContainer.gamepadEx1.rightStickY() < -0.1) {
            turretPos -= (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx1.rightStickY(), 2);
        }

        // TODO: Remove in the future and clamp inside the function instead
        if (turretPos > 1) {
            turretPos = 1;
        } else if (turretPos < -1) {
            turretPos = -1;
        }

        robotContainer.turret.setTargetPosition(turretPos);

        if (robotContainer.gamepadEx1.triangle.isHeld()) {
            // constants for motor speed, different speed based off of position
            RobotContainer.HardwareDevices.flyWheelMotorMaster.setPower(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_SPEED));
            RobotContainer.HardwareDevices.flyWheelMotorSlave.setPower(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_SPEED));
        } else {
            RobotContainer.HardwareDevices.flyWheelMotorMaster.setPower(0);
            RobotContainer.HardwareDevices.flyWheelMotorSlave.setPower(0);
        }

        if (robotContainer.gamepadEx1.circle.wasJustPressed()) {
            Status.intakeEnabled = !Status.intakeEnabled;
        }

        if (Status.intakeEnabled) {
            robotContainer.intake.setIntakeVelocity(1);
        } else {
            robotContainer.intake.setIntakeVelocity(0);
        }

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
