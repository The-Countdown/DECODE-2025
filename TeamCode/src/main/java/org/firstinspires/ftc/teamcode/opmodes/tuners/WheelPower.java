package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import org.firstinspires.ftc.teamcode.main.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "WheelPower", group = "Tuner")
public class WheelPower extends OpMode {
    private RobotContainer robotContainer;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED.LED_COLOR.GREEN);
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this);
        Status.opModeIsActive = true;
        if (RobotContainer.HardwareDevices.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            robotContainer.addRetainedTelemetry("WARNING, PINPOINT STATUS:", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus());
        }

        for (int i = 0; i < Constants.Swerve.NUM_MOTORS; i++) {
            if (robotContainer.swerveModules[i].getPowerMultiplier() != 0) {
                // Are you sure you want to continue.
            }
        }
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();



        // Drive forward
        // Set swerve wheel orientation to zero
        // Read the front back deadwheel
        // Read the left right deadwheel
        // start power multiplier at 50%
        // if listing left increase the power for the left two wheels
        // if both powers are one and still listing left decrease one, if problem gets worse switch the one to decrease, else continue
        // repeat for right
        // if at x num of inches forward reverse direction
        // log motor multiplier for all wheels
        // by the one wheel should be a 100% and all other wheel should find there multiplier to be at max speed
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
