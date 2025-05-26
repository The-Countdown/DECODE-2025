package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SwervePIDFTuner", group = "TeleOp")
public class SwervePIDFTuner extends OpMode {
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
            for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
                robotContainer.swerveModules[i].servo.setTargetAngle(90);
            }
        } else if (targetTimer.seconds() >= 5 && targetTimer.seconds() < 10) {
            for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
                robotContainer.swerveModules[i].servo.setTargetAngle(-90);
            }
        } else {
            targetTimer.reset();
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
        robotContainer.opMode.telemetry.addData("Servo Target 0", robotContainer.swerveServosPIDF[0].getTargetAngle());
        robotContainer.opMode.telemetry.addData("Servo Target 1", robotContainer.swerveServosPIDF[1].getTargetAngle());
        robotContainer.opMode.telemetry.addData("Servo Target 2", robotContainer.swerveServosPIDF[2].getTargetAngle());
        robotContainer.opMode.telemetry.addData("Servo Target 3", robotContainer.swerveServosPIDF[3].getTargetAngle());
        robotContainer.opMode.telemetry.addLine();
        robotContainer.displayRetainedTelemetry();
        robotContainer.opMode.telemetry.update();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
