package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
    public static boolean fieldOriented = false;
    public static double CURRENT_LOOP_TIME_MS = 0;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLight.setColor(Constants.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.drivetrainSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
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
        resetRuntime();
        robotContainer.refreshData();
        gamepadEx1.update();
        gamepadEx2.update();

        robotContainer.drivetrain.drivetrainDirectionalInput(
                robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_x),
                robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_y),
                robotContainer.drivetrain.joystickScaler(gamepad1.right_stick_x),
                fieldOriented
        );

        RobotContainer.HardwareDevices.indicatorLight.setPosition(gamepadEx1.rightTriggerRaw());

        robotContainer.opMode.telemetry.clear();
        robotContainer.opMode.telemetry.addData("Voltage:", robotContainer.getVoltage() + "V");
        robotContainer.opMode.telemetry.addData("Current:", robotContainer.getCurrent() + "A");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Pinpoint X:", PinpointUpdater.currentPose.getX(DistanceUnit.CM) + "cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Y:", PinpointUpdater.currentPose.getY(DistanceUnit.CM) + "cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Heading:", PinpointUpdater.currentHeading + "°");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Loop Time:", CURRENT_LOOP_TIME_MS + "ms");
        robotContainer.opMode.telemetry.update();

        RobotContainer.HardwareDevices.pinpoint.update();

        CURRENT_LOOP_TIME_MS = getRuntime() * 1000;
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.drivetrainSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.isRunning = false;
    }
}
