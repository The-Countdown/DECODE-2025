package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "SlippageTest", group = "Auto")
public class SlippageTest extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static boolean fieldOriented = false;
    public static double CURRENT_LOOP_TIME_MS;
    private static final ElapsedTime turretAccelerationTimer = new ElapsedTime();
    private static final ElapsedTime rotateTimer = new ElapsedTime();
    private int currentServo = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
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
        Status.lightsOn = false;
        Status.isDrivingActive = false;

        turretAccelerationTimer.reset();
        for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
            robotContainer.swerveModules[i].servo.setPower(1);
        }
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();

//        if (rotateTimer.seconds() > 2) {
//            for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
//                robotContainer.swerveModules[i].servo.setPower(-1);
//            }
//            rotateTimer.reset();
//        } else if (rotateTimer.seconds() > 1) {
//            for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
//                robotContainer.swerveModules[i].servo.setPower(1);
//            }
//        }

        if (gamepad1.dpad_up) {
            currentServo = 0;
        } else if (gamepad1.dpad_right) {
            currentServo = 1;
        } else if (gamepad1.dpad_down) {
            currentServo = 2;
        } else if (gamepad1.dpad_left) {
            currentServo = 3;
        }

        robotContainer.telemetry(currentServo, 0, CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS);

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
