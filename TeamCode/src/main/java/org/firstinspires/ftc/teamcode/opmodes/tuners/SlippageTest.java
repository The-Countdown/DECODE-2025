package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "SlippageTest", group = "Auto")
public class SlippageTest extends OpMode {
    private RobotContainer robotContainer;
    private static final ElapsedTime turretAccelerationTimer = new ElapsedTime();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
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
        robotContainer.start(this);

        Status.opModeIsActive = true;
        Status.lightsOn = false;
        Status.isDrivingActive = false;

        turretAccelerationTimer.reset();
        for (int i = 0; i < Constants.Swerve.NUM_SERVOS; i++) {
            robotContainer.swerveModules[i].servo.setPower(1);
        }
    }

    @Override
    public void loop() {
        robotContainer.updateLoopTime("teleOp");
        robotContainer.refreshData();

        robotContainer.telemetry("teleOp");

        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
