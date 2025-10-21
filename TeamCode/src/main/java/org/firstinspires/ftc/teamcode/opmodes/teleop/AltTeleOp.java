package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensFunctions;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AltTeleOp", group = "TeleOp")
public class AltTeleOp extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    public double turretPos = 0;
    public boolean beamBreakBoolean;
    public GamepadWrapper.ButtonReader beamBreakButton = new GamepadWrapper.ButtonReader();

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
        robotContainer.start(this);
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
        beamBreakButton.update(beamBreakBoolean);

        if (robotContainer.gamepadEx1 != null && robotContainer.gamepadEx1.cross.isHeld()) {
            // constants for motor speed, different speed based off of position
            RobotContainer.HardwareDevices.flyWheelMotorMaster.setPower(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_SPEED));
            RobotContainer.HardwareDevices.flyWheelMotorSlave.setPower(Math.min(robotContainer.gamepadEx1.cross.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, Constants.Turret.FLYWHEEL_SPEED));
        } else {
            RobotContainer.HardwareDevices.flyWheelMotorMaster.setPower(0);
            RobotContainer.HardwareDevices.flyWheelMotorSlave.setPower(0);
        }

        robotContainer.transfer.setLowPower(robotContainer.gamepadEx1.leftTriggerRaw());
        robotContainer.transfer.setHighPower(robotContainer.gamepadEx1.rightTriggerRaw());

        robotContainer.telemetry.addData("flywheel speed", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.addData("flywheel current mA", RobotContainer.HardwareDevices.flyWheelMotorMaster.getCurrent(CurrentUnit.MILLIAMPS));
        robotContainer.telemetry.addData("upper flywheel speed", RobotContainer.HardwareDevices.flyWheelMotorSlave.getVelocity());
        robotContainer.telemetry.addData("upper flywheel current mA", RobotContainer.HardwareDevices.flyWheelMotorSlave.getCurrent(CurrentUnit.MILLIAMPS));


        if (robotContainer.gamepadEx1 != null && robotContainer.gamepadEx1.circle.isHeld()) {
            robotContainer.intake.setIntakeVelocity(Math.min(robotContainer.gamepadEx1.circle.getHoldDuration(), 1));
        }
        else {
            robotContainer.intake.setIntakeVelocity(0);
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

        robotContainer.limelightLogic.trackGoal();

        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.pointAtGoal(), () -> beamBreakBoolean);
        robotContainer.turret.pointAtGoal();

//        robotContainer.turret.setTurretTargetPosition(Math.min(Math.max(turretPos, -1), 1));
        robotContainer.telemetry.addData("turret pos",turretPos);

//        RobotContainer.HardwareDevices.spindexServo.setPower(robotContainer.gamepadEx1.leftStickX() * 0.2);
        beamBreakBoolean = RobotContainer.HardwareDevices.beamBreak.isPressed();

//        if (beamBreakButton.getHoldDuration() > Constants.INTAKE_DELAY_SECONDS) {
//            robotContainer.intake.setIntakeVelocity(0.9);
//            robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setIntakeVelocity(0), Constants.INTAKE_RUNTIME_MS);
//        }
        robotContainer.telemetry.addData("beam break", beamBreakBoolean);
        robotContainer.telemetry.update();

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
