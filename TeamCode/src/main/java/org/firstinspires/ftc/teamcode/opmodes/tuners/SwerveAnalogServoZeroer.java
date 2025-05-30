package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SwerveAnalogServoZeroer", group = "Tuner")
public class SwerveAnalogServoZeroer extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    private int currentServo = -1;
    private double[] offset = {0, 0, 0, 0};

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
        robotContainer.start(this);
        Status.opModeIsActive = true;
        Objects.requireNonNull(robotContainer.loopTimers.get("teleOp")).reset();
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpoint.DeviceStatus.READY) {
            robotContainer.addRetainedTelemetry("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
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

        if (currentServo >= 0) {
            if (gamepad1.right_bumper) {
                robotContainer.swerveModules[currentServo].servo.setTargetAngle(robotContainer.swerveServosPIDF[currentServo].getTargetAngle() + 0.3);
                offset[currentServo] += 0.3;
            } else if (gamepad1.left_bumper) {
                robotContainer.swerveModules[currentServo].servo.setTargetAngle(robotContainer.swerveServosPIDF[currentServo].getTargetAngle() - 0.3);
                offset[currentServo] -= 0.3;
            }
            robotContainer.telemetry(currentServo, offset[currentServo], CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS, gamepad1);
        }

        robotContainer.allIndicatorLights.off();

    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
