package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Rotate", group = "Auto")
public class Rotate extends OpMode {
    private RobotContainer robotContainer;
    private static final ElapsedTime rotateTimer = new ElapsedTime();
    double[] angles = {0, 0, 0, 0};
    double angle = 0;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.telemetry.addLine("OpMode Initialized");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this, false);
        Status.opModeIsActive = true;
        Status.lightsOn = false;
        Status.isDrivingActive = false;
    }

    @Override
    public void loop() {
        robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();

        if (rotateTimer.seconds() > 1) {
            for (int i = 0; i < angles.length; i++) {
                angles[i] += 10;
            }
            angle += 10;
            rotateTimer.reset();
        }

        if (angle > 360) {
            Arrays.fill(angles, 0);
            angle = 0;
        }

        robotContainer.drivetrain.setTargets(angles, Constants.Swerve.NO_POWER);

        robotContainer.telemetry("teleOp");
        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
