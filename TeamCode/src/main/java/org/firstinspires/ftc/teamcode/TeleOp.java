package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private final RobotManager robotManager = new RobotManager(this);
    public static boolean fieldOriented = false;

    @Override
    public void init() {
        robotManager.isRunning = true;
        robotManager.refreshData();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robotManager.loopTime.reset();
        robotManager.drivetrain.driverControl(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, fieldOriented);

        robotManager.opMode.telemetry.addData("Voltage", robotManager.getVoltage());
        robotManager.opMode.telemetry.addData("Current", robotManager.getCurrent());
        robotManager.opMode.telemetry.addData("Robot Yaw", ThreadedIMU.currentYaw);
        robotManager.opMode.telemetry.addData("Loop Time:", robotManager.loopTime.milliseconds());
        robotManager.opMode.telemetry.update();
    }

    @Override
    public void stop() {
        robotManager.drivetrain.drivetrainInput(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotManager.isRunning = false;
    }
}
