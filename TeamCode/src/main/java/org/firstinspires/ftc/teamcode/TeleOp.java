package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@SuppressWarnings("all")
public class TeleOp extends OpMode {
    private final Robot robot = new Robot(this);

    @Override
    public void init() {
        robot.isRunning = true;
        robot.refreshData();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.drivetrain.driverControl(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false);

        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Current", robot.getCurrent());
        telemetry.addData("Robot Yaw", ThreadedIMU.currentYaw);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drivetrain.drivetrainInput(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robot.isRunning = false;
    }
}
