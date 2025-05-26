package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainUpdater;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
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
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.opMode.telemetry.setMsTransmissionInterval(750);
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        gamepadEx1 = new GamepadWrapper(gamepad1);
        gamepadEx2 = new GamepadWrapper(gamepad2);

        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;

        robotContainer.pinpointUpdater = new PinpointUpdater(robotContainer);
        robotContainer.pinpointUpdater.start();
        robotContainer.drivetrainUpdater = new DrivetrainUpdater(robotContainer);
        robotContainer.drivetrainUpdater.start();

        turretAccelerationTimer.reset();
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        gamepadEx1.update();
        gamepadEx2.update();

        if (gamepad1.dpad_up) {
            currentServo = 0;
        } else if (gamepad1.dpad_right) {
            currentServo = 1;
        } else if (gamepad1.dpad_down) {
            currentServo = 2;
        } else if (gamepad1.dpad_left) {
            currentServo = 3;
        }

        if (Status.isDrivingActive) {
            if (Constants.MECANUM_ACTIVE) {
                robotContainer.drivetrain.mecanumDrive(
                        robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_y),
                        robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_x),
                        robotContainer.drivetrain.joystickScaler(gamepad1.right_stick_x),
                        gamepad1.right_trigger,
                        gamepad1.left_trigger
                );
            } else {
                robotContainer.drivetrain.swerveDirectionalInput(
                        robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_x),
                        robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_y),
                        robotContainer.drivetrain.joystickScaler(gamepad1.right_stick_x),
                        fieldOriented
                );
            }
        }

        if (Constants.TURRET_ACTIVE) {
            if (gamepad1.right_bumper) {
                robotContainer.turret.setTurretSpinPower(0.6);
            } else if (gamepad1.left_bumper) {
                robotContainer.turret.setTurretSpinPower(-0.6);
            } else {
                robotContainer.turret.setTurretSpinPower(0);
            }

            if (gamepadEx1.a.wasJustPressed() || gamepadEx1.b.wasJustPressed()) {
                turretAccelerationTimer.reset();
            }
            if (gamepadEx1.a.isHeld()) {
                robotContainer.turret.flywheel.setVelocity(
                        turretAccelerationTimer.nanoseconds()
                                * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO
                );
            } else if (gamepadEx1.b.isHeld()) {
                robotContainer.turret.flywheel.setVelocity(
                        -turretAccelerationTimer.nanoseconds()
                                * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO
                );
            } else {
                robotContainer.turret.flywheel.setPower(0);
            }
        }

        if (gamepadEx1.guide.isHeldFor(2) && Status.lightsOn) {
            robotContainer.indicatorLightBack.flashingReset();
            robotContainer.indicatorLightFrontLeft.flashingReset();
            robotContainer.indicatorLightFrontRight.flashingReset();
            Status.lightsOn = false;
            robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
            Status.isDrivingActive = false;
        }

        if (gamepadEx1.guide.wasJustPressed() && !Status.lightsOn) {
            Status.lightsOn = true;
            robotContainer.delayedActionManager.schedule(() -> Status.isDrivingActive = true, 1000);
        }

        if (!Status.lightsOn) {
            robotContainer.indicatorLightBack.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.OFF, 8, 2);
            robotContainer.indicatorLightFrontLeft.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.OFF, 8, 2);
            robotContainer.indicatorLightFrontRight.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.OFF, 8, 2);
        }

        if (Status.lightsOn) {
            if (gamepad1.left_stick_x > 0.1) {
                robotContainer.indicatorLightFrontRight.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.WHITE, 2);
                robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.WHITE);
            } else if (gamepad1.left_stick_x < -0.1) {
                robotContainer.indicatorLightFrontLeft.flashing(Constants.LED_COLOR.ORANGE, Constants.LED_COLOR.WHITE, 2);
                robotContainer.indicatorLightFrontRight.setColor(Constants.LED_COLOR.WHITE);
            } else {
                robotContainer.indicatorLightFrontRight.setColor(Constants.LED_COLOR.WHITE);
                robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.WHITE);
            }

            if (gamepad1.left_stick_y < -0.1) {
                robotContainer.indicatorLightBack.rainbow();
            } else if (gamepad1.left_stick_y > 0.1) {
                robotContainer.indicatorLightBack.flashing(Constants.LED_COLOR.RED, Constants.LED_COLOR.WHITE, 2);
            } else {
                robotContainer.indicatorLightBack.setColor(Constants.LED_COLOR.RED);
            }

            if (gamepadEx1.leftStickY.wasJustReleased()) {
                robotContainer.indicatorLightBack.rainbowReset();
            }
        }

        robotContainer.opMode.telemetry.addData("Control Hub Voltage", robotContainer.getVoltage(Constants.CONTROL_HUB_INDEX) + " V");
        robotContainer.opMode.telemetry.addData("Expansion Hub Voltage", robotContainer.getVoltage(Constants.EXPANSION_HUB_INDEX) + " V");
        robotContainer.opMode.telemetry.addData("Control Hub Current", robotContainer.getCurrent(Constants.CONTROL_HUB_INDEX) + " A");
        robotContainer.opMode.telemetry.addData("Expansion Hub Current", robotContainer.getCurrent(Constants.EXPANSION_HUB_INDEX) + " A");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Pinpoint X", PinpointUpdater.currentPose.getX(DistanceUnit.CM) + " cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Y", PinpointUpdater.currentPose.getY(DistanceUnit.CM) + " cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Heading", PinpointUpdater.currentHeading + "°");
        robotContainer.opMode.telemetry.addData("PINPOINT STATUS", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("TeleOp Avg Loop Time", (int) CURRENT_LOOP_TIME_AVG_MS + " ms");
        robotContainer.opMode.telemetry.addData("TeleOp Loop Time", (int) CURRENT_LOOP_TIME_MS + " ms");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("DriveTrain Avg Loop Time", (int) robotContainer.drivetrainUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        robotContainer.opMode.telemetry.addData("DriveTrain Loop Time", (int) robotContainer.drivetrainUpdater.CURRENT_LOOP_TIME_MS + " ms");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Pinpoint Avg Loop Time", (int) robotContainer.pinpointUpdater.CURRENT_LOOP_TIME_AVG_MS + " ms");
        robotContainer.opMode.telemetry.addData("Pinpoint Loop Time", (int) robotContainer.pinpointUpdater.CURRENT_LOOP_TIME_MS + " ms");
        if (currentServo >= 0) {
            robotContainer.opMode.telemetry.addLine();
            robotContainer.opMode.telemetry.addData("Selected Servo", currentServo);
            robotContainer.opMode.telemetry.addData("Servo Angle", robotContainer.swerveModules[currentServo].servo.getAngle());
            robotContainer.opMode.telemetry.addData("Servo Target", robotContainer.swerveServosPIDF[currentServo].getTargetAngle());
            robotContainer.opMode.telemetry.addData("Servo Set Power", robotContainer.swerveServosPIDF[currentServo].calculate());
            robotContainer.opMode.telemetry.addData("Servo Error", robotContainer.swerveServosPIDF[currentServo].getError());
            robotContainer.opMode.telemetry.addData("Motor Target Power", robotContainer.swerveModules[currentServo].motor.targetPower);
            robotContainer.opMode.telemetry.addData("Motor Current Power", RobotContainer.HardwareDevices.swerveMotors[currentServo].getPower());
        }
        robotContainer.displayRetainedTelemetry();
        robotContainer.opMode.telemetry.update();

        Thread.yield();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
