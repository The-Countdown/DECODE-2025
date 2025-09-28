package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.ADG728;
import org.firstinspires.ftc.teamcode.other.ADGUpdater;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AltTeleOp", group = "TeleOp")
public class AltTeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        RobotContainer.HardwareDevices.limelight.pipelineSwitch(0);
        robotContainer.telemetry.addLine("OpMode Initialized");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;
        RobotContainer.HardwareDevices.limelight.start();
        robotContainer.start(this);

        robotContainer.drivetrainUpdater.enabled = false;
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        if (robotContainer.gamepadEx1 != null){
            robotContainer.gamepadEx1.update();
        }
        if (robotContainer.gamepadEx2 != null){
            robotContainer.gamepadEx2.update();
        }
        robotContainer.limelightLogic.updateLimelight();

//        RobotContainer.HardwareDevices.mux1.setPortRaw(254);
//        RobotContainer.HardwareDevices.mux1.saveVoltage();
        robotContainer.telemetry.addData("Sensors: ", Arrays.toString(RobotContainer.HardwareDevices.mux1.getVolteges()));

        telemetry.update();

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
