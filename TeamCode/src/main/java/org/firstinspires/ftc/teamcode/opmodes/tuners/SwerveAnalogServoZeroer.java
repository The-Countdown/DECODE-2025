package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SwerveAnalogServoZeroer", group = "TeleOp")
public class SwerveAnalogServoZeroer extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
    public static double CURRENT_LOOP_TIME_MS;
    private int currentServo = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.opMode.telemetry.setMsTransmissionInterval(200);
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.GREEN);
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
        robotContainer.loopTimer.reset();
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpoint.DeviceStatus.READY) {
            robotContainer.addRetainedTelemetry("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTimeTracking();
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime();
        robotContainer.refreshData();
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

        if (currentServo >= 0) {
            if (gamepad1.right_bumper) {
                robotContainer.swerveModules[currentServo].servo.setTargetAngle(robotContainer.swerveServosPIDF[currentServo].getTargetAngle() + 0.3);
            } else if (gamepad1.left_bumper) {
                robotContainer.swerveModules[currentServo].servo.setTargetAngle(robotContainer.swerveServosPIDF[currentServo].getTargetAngle() - 0.3);
            }
        }

        robotContainer.indicatorLightFrontLeft.off();

        robotContainer.opMode.telemetry.addData("Control Hub Voltage", robotContainer.getVoltage(Constants.CONTROL_HUB_INDEX) + " V");
        robotContainer.opMode.telemetry.addData("Expansion Hub Voltage", robotContainer.getVoltage(Constants.EXPANSION_HUB_INDEX) + " V");
        robotContainer.opMode.telemetry.addData("Control Hub Current", robotContainer.getCurrent(Constants.CONTROL_HUB_INDEX) + " A");
        robotContainer.opMode.telemetry.addData("Expansion Hub Current", robotContainer.getCurrent(Constants.EXPANSION_HUB_INDEX) + " A");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Pinpoint X", PinpointUpdater.currentPose.getX(DistanceUnit.CM) + " cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Y", PinpointUpdater.currentPose.getY(DistanceUnit.CM) + " cm");
        robotContainer.opMode.telemetry.addData("Pinpoint Heading", PinpointUpdater.currentHeading + "°");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Avg Loop Time", (int) CURRENT_LOOP_TIME_AVG_MS + " ms");
        robotContainer.opMode.telemetry.addData("Loop Time", (int) CURRENT_LOOP_TIME_MS + " ms");
        robotContainer.opMode.telemetry.addLine();
        robotContainer.opMode.telemetry.addData("Selected Servo", currentServo);
        if (currentServo >= 0) {
            robotContainer.opMode.telemetry.addData("Servo Angle", robotContainer.swerveModules[currentServo].servo.getAngle());
            robotContainer.opMode.telemetry.addData("Servo Target", robotContainer.swerveServosPIDF[currentServo].getTargetAngle());
            robotContainer.opMode.telemetry.addData("Servo Set Power", robotContainer.swerveServosPIDF[currentServo].calculate());
            robotContainer.opMode.telemetry.addData("Servo Error", robotContainer.swerveServosPIDF[currentServo].getError());
        }
        robotContainer.displayRetainedTelemetry();
        robotContainer.opMode.telemetry.update();

        RobotContainer.HardwareDevices.pinpoint.update();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
