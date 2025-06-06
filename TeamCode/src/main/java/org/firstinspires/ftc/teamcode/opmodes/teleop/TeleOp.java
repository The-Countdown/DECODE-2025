package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static boolean fieldOriented = false;
    public static double CURRENT_LOOP_TIME_MS;
    private static final ElapsedTime turretAccelerationTimer = new ElapsedTime();
    private int currentServo = -1;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.opMode.telemetry.setMsTransmissionInterval(750);
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
        Status.lightsOn = true;
        Status.isDrivingActive = true;

        turretAccelerationTimer.reset();
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (gamepad1.dpad_up) {
            currentServo = 0;
        } else if (gamepad1.dpad_right) {
            currentServo = 1;
        } else if (gamepad1.dpad_down) {
            currentServo = 2;
        } else if (gamepad1.dpad_left) {
            currentServo = 3;
        }

        if (Status.isDrivingActive) {
            robotContainer.drivetrain.mecanumDrive(
                    robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_y),
                    robotContainer.drivetrain.joystickScaler(gamepad1.left_stick_x),
                    robotContainer.drivetrain.joystickScaler(gamepad1.right_stick_x),
                    gamepad1.right_trigger,
                    gamepad1.left_trigger
                    );
        }

        if (gamepad1.right_bumper) {
            robotContainer.turret.setTurretSpinPower(0.6);
        } else if (gamepad1.left_bumper) {
            robotContainer.turret.setTurretSpinPower(-0.6);
        } else {
            robotContainer.turret.setTurretSpinPower(0);
        }

        if (robotContainer.gamepadEx1.a.wasJustPressed() || robotContainer.gamepadEx1.b.wasJustPressed()) {
            turretAccelerationTimer.reset();
        }
        if (robotContainer.gamepadEx1.a.isHeld()) {
            robotContainer.turret.flywheel.setPower(
                    turretAccelerationTimer.nanoseconds()
                    * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO * 4
                    );
        } else if (robotContainer.gamepadEx1.b.isHeld()) {
            robotContainer.turret.flywheel.setPower(
                    -turretAccelerationTimer.nanoseconds()
                    * Constants.TURRET_ACCELERATION_MULTIPLIER_NANO * 4
                    );
        } else {
            robotContainer.turret.flywheel.setPower(0);
        }

        robotContainer.telemetry(currentServo, 0, CURRENT_LOOP_TIME_MS, CURRENT_LOOP_TIME_AVG_MS, gamepad1);

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
