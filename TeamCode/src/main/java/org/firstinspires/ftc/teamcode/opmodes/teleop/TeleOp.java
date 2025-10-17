package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    public static boolean fieldOriented = false;
    private int currentServo = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.telemetry.addLine("OpMode Initialized");
        robotContainer.telemetry.update();
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (gamepad1.dpad_up) {
            currentServo = 0;
        } else if (gamepad1.dpad_right) {
            currentServo = 1;
        } else if (gamepad1.dpad_down) {
            currentServo = 2;
        } else if (gamepad1.dpad_left) {
            currentServo = 3;
        }

        if (!fieldOriented && robotContainer.gamepadEx1.cross.wasJustPressed()) {
            fieldOriented = true;
        } else if (fieldOriented && robotContainer.gamepadEx1.cross.wasJustPressed()) {
            fieldOriented = false;
        }

        if (Status.isDrivingActive) {
            if (Constants.useHeadingPIDForTurning) {
                robotContainer.drivetrain.swerveDirectionalInput(
                        robotContainer.drivetrain.joystickScaler(-robotContainer.gamepadEx1.leftStickX()),
                        robotContainer.drivetrain.joystickScaler(robotContainer.gamepadEx1.leftStickY()),
                        0,
                        fieldOriented
                        );
            } else {
                robotContainer.drivetrain.swerveDirectionalInput(
                        robotContainer.drivetrain.joystickScaler(-robotContainer.gamepadEx1.leftStickX()),
                        robotContainer.drivetrain.joystickScaler(robotContainer.gamepadEx1.leftStickY()),
                        robotContainer.drivetrain.joystickScaler(-robotContainer.gamepadEx1.rightStickX()),
                        fieldOriented
                        );
            }
        }

        if (Constants.useHeadingPIDForTurning) {
            if (robotContainer.gamepadEx1.rightStickX() > 0.05) {
                robotContainer.headingPID.setTargetHeading((robotContainer.headingPID.getTargetHeading() - Constants.turningRate * CURRENT_LOOP_TIME_MS * robotContainer.gamepadEx1.rightStickX()));
            }
            if (robotContainer.gamepadEx1.rightStickX() < -0.05) {
                robotContainer.headingPID.setTargetHeading((robotContainer.headingPID.getTargetHeading() - Constants.turningRate * CURRENT_LOOP_TIME_MS * robotContainer.gamepadEx1.rightStickX()));
            }
        }

        robotContainer.allIndicatorLights.lightsUpdater();

        robotContainer.telemetry(currentServo, 0, CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS);

        Thread.yield();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);

        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
