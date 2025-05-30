package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HeadingPIDTuner", group = "Tuner")
public class HeadingPIDTuner extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    private final ElapsedTime targetTimer = new ElapsedTime();

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
        targetTimer.reset();
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (targetTimer.seconds() < 5) {
            robotContainer.headingPID.setTargetHeading(90);
        } else if (targetTimer.seconds() >= 5 && targetTimer.seconds() < 10) {
            robotContainer.headingPID.setTargetHeading(-90);
        } else {
            targetTimer.reset();
        }

        double power = robotContainer.headingPID.calculate(PinpointUpdater.currentHeading);
        double[] powers = {power,power,power,power};
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_ROTATION_FORMATION_DEGREES, powers);

        robotContainer.allIndicatorLights.off();

        robotContainer.telemetry(-1, 0, CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS, gamepad1);
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
