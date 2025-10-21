package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SwerveAnalogServoZeroer", group = "Tuner")
public class SwerveAnalogServoZeroer extends OpMode {
    private RobotContainer robotContainer;
    private int currentServo = -1;
    private final double[] offset = {0, 0, 0, 0};
    private final double[] power = {0.2, 0.2, 0.2, 0.2};

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLightFront.setColor(Constants.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLightFront.setColor(Constants.LED_COLOR.GREEN);
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this);
        Status.opModeIsActive = true;
        robotContainer.drivetrain.setTargets(offset, power);
        Objects.requireNonNull(robotContainer.loopTimers.get("teleOp")).reset();
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            robotContainer.addRetainedTelemetry("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }
    }

    @Override
    public void loop() {
        robotContainer.updateLoopTime("teleOp");
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
            robotContainer.telemetry("teleOp");
        }

        robotContainer.allIndicatorLights.off();

    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
