package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
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

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLight.setColor(Constants.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        RobotContainer.HardwareDevices.turretRotation.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        robotContainer.opMode.telemetry.setMsTransmissionInterval(200);
        robotContainer.opMode.telemetry.speak("TeleOp Initialized", "en", "GB");
        robotContainer.opMode.telemetry.addLine("TeleOp Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLight.setColor(Constants.LED_COLOR.GREEN);
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        gamepadEx1 = new GamepadWrapper(gamepad1);
        gamepadEx2 = new GamepadWrapper(gamepad2);
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpoint.DeviceStatus.READY) {
            robotContainer.addRetained("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTimeTracking();
        robotContainer.refreshData();
        gamepadEx1.update();
        gamepadEx2.update();

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

        if (gamepad1.right_bumper) {
            robotContainer.turret.setTurretSpinPower(0.6);
        } else if (gamepad1.left_bumper) {
            robotContainer.turret.setTurretSpinPower(-0.6);
        } else {
            robotContainer.turret.setTurretSpinPower(0);
        }

        if (gamepad1.cross) {
            robotContainer.intake.setPower(1);
        } else if (gamepad1.circle) {
            robotContainer.intake.setPower(-1);
        } else {
            robotContainer.intake.setPower(0);
        }

        robotContainer.indicatorLight.rainbow();

        robotContainer.opMode.telemetry.clear();
        robotContainer.opMode.telemetry.addData("Control Hub Voltage:", robotContainer.getVoltage(Constants.CONTROL_HUB_INDEX) + "V");
        robotContainer.opMode.telemetry.addData("Expansion Hub Voltage:", robotContainer.getVoltage(Constants.EXPANSION_HUB_INDEX) + "V");
        robotContainer.opMode.telemetry.addData("Control Hub Current:", robotContainer.getCurrent(Constants.CONTROL_HUB_INDEX) + "A");
        robotContainer.opMode.telemetry.addData("Expansion Hub Current:", robotContainer.getCurrent(Constants.EXPANSION_HUB_INDEX) + "A");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Pinpoint X:", PinpointUpdater.currentPose.getX(DistanceUnit.CM) + "cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Y:", PinpointUpdater.currentPose.getY(DistanceUnit.CM) + "cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Heading:", PinpointUpdater.currentHeading + "°");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Loop Time:", CURRENT_LOOP_TIME_MS + "ms");
        robotContainer.opMode.telemetry.update();

        RobotContainer.HardwareDevices.pinpoint.update();

        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.isRunning = false;
    }
}