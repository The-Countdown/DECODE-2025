package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MaxSpeedTuner", group = "TeleOp")
public class MaxSpeedTuner extends OpMode {
    private RobotContainer robotContainer;
    private final GamepadWrapper.ButtonReader transferConditionButton = new GamepadWrapper.ButtonReader();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private double currentSpeed = 0;
    private double topSpeed = 0;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        for (int i = 0; i < Constants.Swerve.NUM_SERVOS; i++) {
            robotContainer.swerveModules[i].servo.setTargetAngle(90);
        }
        Status.opModeIsActive = true;
    }

    @Override
    public void init_loop() {
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        robotContainer.start(this, true);
        Status.isDrivingActive = true;

        for (int i = 0; i < Constants.Swerve.NUM_SERVOS; i++) {
            robotContainer.swerveModules[i].motor.setTargetPower(0.8);
        }
    }

    @Override
    public void loop() {
        robotContainer.CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();

        currentSpeed = (robotContainer.swerveModules[0].motor.getVelocity() +
        robotContainer.swerveModules[1].motor.getVelocity() +
        robotContainer.swerveModules[2].motor.getVelocity() +
        robotContainer.swerveModules[3].motor.getVelocity()) / 4;

        if(currentSpeed > topSpeed){
            topSpeed = currentSpeed;
        }
        robotContainer.telemetry.addData("Max Speed: ", topSpeed);




        robotContainer.telemetry.update();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);

        Status.isDrivingActive = false;
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
