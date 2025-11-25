package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AltTeleOp", group = "TeleOp")
public class AltTeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    public double turretPos = 0;

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

        robotContainer.turret.setTargetAngle(177.5);
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
        robotContainer.start(this, false);
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
        robotContainer.drivetrain.controlUpdate();

        if (robotContainer.gamepadEx1.cross.isHeld()) {
            // constants for motor speed, different speed based off of position
            // robotContainer.turret.flywheel.setTargetVelocity(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_TOP_SPEED));
        } else {
            // robotContainer.turret.flywheel.setTargetVelocity(0);
        }

        //transfer test (make automated once it works)
        if (robotContainer.gamepadEx1.square.wasJustPressed()) {
            robotContainer.transfer.flapUp();
            robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), Constants.Transfer.FLIP_TIME);
        }

        if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
            robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
        }


//        robotContainer.transfer.setHighPower(robotContainer.gamepadEx1.rightTriggerRaw());

        if (robotContainer.gamepadEx1.circle.isHeld()) {
            robotContainer.intake.setPower(Math.min(robotContainer.gamepadEx1.circle.getHoldDuration(), 1));
        }
        else {
            robotContainer.intake.setPower(0);
        }

        //turret speed factor * current loop time is how far u want it to move per how many millisecond(loop time)
        if (robotContainer.gamepadEx1.rightStickX() > 0.1) {
            turretPos += (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx1.rightStickX(), 2);
        } else if (robotContainer.gamepadEx1.rightStickX() < -0.1) {
            turretPos -= (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx1.rightStickX(), 2);
        }

        if (turretPos > 1) {
            turretPos = 1;
        } else if (turretPos < -1) {
            turretPos = -1;
        }

        robotContainer.turret.setTargetPosition(Math.min(Math.max(turretPos, -1), 1));

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
