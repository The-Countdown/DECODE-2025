package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve Forward Tuner", group = "Tuner")
public class SwerveForwardTuner extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    private final ElapsedTime targetTimer = new ElapsedTime();
    public boolean direction = true;
    long timer = System.currentTimeMillis();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this);
        Status.opModeIsActive = true;
        targetTimer.reset();

        robotContainer.localizationUpdater.start();

        robotContainer.swerveModules[0].servo.setTargetAngle(90);
        robotContainer.swerveModules[1].servo.setTargetAngle(90);
        robotContainer.swerveModules[2].servo.setTargetAngle(90);
        robotContainer.swerveModules[3].servo.setTargetAngle(90);
    }

    @Override
    public void loop() {
        if (SwerveForwardTunerConstants.HEADING_HOLD) {
            robotContainer.headingPID.setEnabled(true);
        } else {
            robotContainer.headingPID.setEnabled(true);
        }

        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (System.currentTimeMillis() - timer >= SwerveForwardTunerConstants.DIRECTION_SWAP_TIME) {
            // Switch direction every 5 seconds
            if (direction) {
                robotContainer.swerveModules[0].motor.setTargetPower(SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[1].motor.setTargetPower(SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[2].motor.setTargetPower(SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[3].motor.setTargetPower(SwerveForwardTunerConstants.POWER);
                direction = false;
            } else {
                robotContainer.swerveModules[0].motor.setTargetPower(-SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[1].motor.setTargetPower(-SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[2].motor.setTargetPower(-SwerveForwardTunerConstants.POWER);
                robotContainer.swerveModules[3].motor.setTargetPower(-SwerveForwardTunerConstants.POWER);
                direction = true;
            }
            timer = System.currentTimeMillis();
        }
        robotContainer.telemetry.addData("Timer", timer);
        robotContainer.telemetry.addData("Timer Diff", System.currentTimeMillis() - timer);
        robotContainer.telemetry.addData("Current Heading", Status.currentHeading);
        robotContainer.telemetry.addData("Target Heading", robotContainer.headingPID.getTargetHeading());


        robotContainer.telemetry.update();
    }
    
    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }

    @Config
    public static class SwerveForwardTunerConstants {
        public static long DIRECTION_SWAP_TIME = 3000;
        public static double POWER = 0.4;
        public static boolean HEADING_HOLD = false;
    }
}

