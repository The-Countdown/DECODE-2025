package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensFunctions;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AltTeleOp", group = "TeleOp")
public class AltTeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;

    private HuskyLensFunctions lens;

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
        lens = new HuskyLensFunctions(robotContainer, RobotContainer.HardwareDevices.huskyLens1);
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
        robotContainer.telemetry.setMsTransmissionInterval(50);
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
        // lens.checkColor();
        lens.nearestBall();
        /*
        RobotContainer.HardwareDevices.mux1.setActivePortRaw(1);
        RobotContainer.HardwareDevices.mux1.setPortRaw(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RobotContainer.HardwareDevices.mux1.saveVoltage();
        robotContainer.telemetry.addData("vol 1: ", RobotContainer.HardwareDevices.mux1.getVoltageRaw());
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        RobotContainer.HardwareDevices.mux1.setActivePortRaw(2);
        RobotContainer.HardwareDevices.mux1.setPortRaw(2);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        RobotContainer.HardwareDevices.mux1.saveVoltage();
        robotContainer.telemetry.addData("vol 2: ", RobotContainer.HardwareDevices.mux1.getVoltageRaw());
        robotContainer.telemetry.addData("Sensors: ", Arrays.toString(RobotContainer.HardwareDevices.mux1.getVoltages()));

        telemetry.update();
        */

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
