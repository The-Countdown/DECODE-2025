package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SwervePIDFTuner", group = "Tuner")
public class SwervePIDFTuner extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime targetTimer = new ElapsedTime();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED.LED_COLOR.GREEN);
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
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            robotContainer.addRetainedTelemetry("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }
        targetTimer.reset();
    }

    @Override
    public void loop() {
        robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (targetTimer.seconds() < 5) {
            for (int i = 0; i < Constants.Swerve.NUM_SERVOS; i++) {
                robotContainer.swerveModules[i].servo.setTargetAngle(90);
            }
        } else if (targetTimer.seconds() >= 5 && targetTimer.seconds() < 10) {
            for (int i = 0; i < Constants.Swerve.NUM_SERVOS; i++) {
                robotContainer.swerveModules[i].servo.setTargetAngle(-90);
            }
        } else {
            targetTimer.reset();
        }

        robotContainer.allIndicatorLights.off();

        robotContainer.telemetry("teleOp");
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
