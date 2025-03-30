package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@SuppressWarnings("all")
public class TeleOp extends OpMode {
    private final Robot robot = new Robot(this);
    public static boolean fieldOriented = false;

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
        robot.drivetrain.driverControl(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, fieldOriented);

        robot.opMode.telemetry.addData("Voltage", robot.getVoltage());
        robot.opMode.telemetry.addData("Current", robot.getCurrent());
        robot.opMode.telemetry.addData("Robot Yaw", ThreadedIMU.currentYaw);
        robot.opMode.telemetry.update();
    }

    @Override
    public void stop() {
        robot.drivetrain.drivetrainInput(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robot.isRunning = false;
    }
}
