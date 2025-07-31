package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static boolean fieldOriented = false;
    public static double CURRENT_LOOP_TIME_MS;
    private static final ElapsedTime turretAccelerationTimer = new ElapsedTime();
    private int currentServo = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        if (!robotContainer.turretFunctional) {
            robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        }
        robotContainer.telemetry.addLine("OpMode Initialized");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this);

        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;

        turretAccelerationTimer.reset();
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

        if (!fieldOriented && robotContainer.gamepadEx1.a.wasJustPressed()) {
            fieldOriented = true;
        } else if (fieldOriented && robotContainer.gamepadEx1.a.wasJustPressed()) {
            fieldOriented = false;
        }


        if (Status.isDrivingActive) {
            // Fix this for also drivetrain working during turret.
            if (!robotContainer.turretFunctional) {
                robotContainer.drivetrain.swerveDirectionalInput(
                        robotContainer.drivetrain.joystickScaler(-robotContainer.gamepadEx1.leftStickX()),
                        robotContainer.drivetrain.joystickScaler(robotContainer.gamepadEx1.leftStickY()),
                        robotContainer.drivetrain.joystickScaler(-robotContainer.gamepadEx1.rightStickX()),
                        fieldOriented
                );
            }
        }

        if (robotContainer.turretFunctional) {
            if (gamepad1.right_bumper) {
                robotContainer.turret.setTurretSpinPower(0.6);
            } else if (gamepad1.left_bumper) {
                robotContainer.turret.setTurretSpinPower(-0.6);
            } else {
                robotContainer.turret.setTurretSpinPower(0);
            }

            if (robotContainer.gamepadEx1.a.wasJustPressed() || robotContainer.gamepadEx1.b.wasJustPressed()) {
                turretAccelerationTimer.reset();
            }
            if (robotContainer.gamepadEx1.a.isHeld()) {
                robotContainer.turret.flywheel.setVelocity(
                        turretAccelerationTimer.nanoseconds()
                                * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO
                );
            } else if (robotContainer.gamepadEx1.b.isHeld()) {
                robotContainer.turret.flywheel.setVelocity(
                        -turretAccelerationTimer.nanoseconds()
                                * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO
                );
            } else {
                robotContainer.turret.flywheel.setPower(0);
            }
        }

        if (robotContainer.gamepadEx1.guide.isHeldFor(0.75) && Status.lightsOn) {
            robotContainer.allIndicatorLights.flashingReset();
            Status.lightsOn = false;
            if (!robotContainer.turretFunctional) {
                robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
            }
            Status.isDrivingActive = false;
        }

        if (robotContainer.gamepadEx1.guide.wasJustPressed() && !Status.lightsOn) {
            Status.lightsOn = true;
            robotContainer.delayedActionManager.schedule(() -> Status.isDrivingActive = true, 1000);
        }

        if (!Status.lightsOn) {
            robotContainer.allIndicatorLights.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.OFF, 8, 2);
        }

        if (Status.lightsOn) {
            if (robotContainer.gamepadEx1.leftStickX() > 0.1) {
                robotContainer.indicatorLightFrontRight.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.WHITE, 2);
                robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.WHITE);
            } else if (robotContainer.gamepadEx1.leftStickX() < -0.1) {
                robotContainer.indicatorLightFrontLeft.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.WHITE, 2);
                robotContainer.indicatorLightFrontRight.setColor(Constants.LED_COLOR.WHITE);
            } else if (robotContainer.gamepadEx1.leftStickY() > 0.1) {
                robotContainer.allIndicatorLights.rainbow();
            } else {
                robotContainer.indicatorLightFrontRight.setColor(Constants.LED_COLOR.WHITE);
                robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.WHITE);
            }

            if (robotContainer.gamepadEx1.leftStickY() < -0.1) {
                robotContainer.indicatorLightBack.flashing(Constants.LED_COLOR.RED, Constants.LED_COLOR.WHITE, 2);
            } else {
                robotContainer.indicatorLightBack.setColor(Constants.LED_COLOR.RED);
            }

            if (robotContainer.gamepadEx1.leftStickY.wasJustReleased()) {
                robotContainer.allIndicatorLights.rainbowReset();
            }
        }

        robotContainer.telemetry(currentServo, 0, CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS);

        Thread.yield();
    }

    @Override
    public void stop() {
        if (!robotContainer.turretFunctional) {
            robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        }
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
