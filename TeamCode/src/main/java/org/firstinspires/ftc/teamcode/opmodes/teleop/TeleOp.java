package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.graphics.drawable.Drawable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.FeedForward;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathingUpdater;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pinpointTimer = new ElapsedTime();
    private double currentHeading = 0;
    private double currentSpeed = 0;
    private double currentDist = 0;
    private double lastDist = 0;
    private double lastHeading = 0;
    private double htopSpeed = 0;
    private double topSpeed = 0;
    private double hvelocity = 0;
    private final ElapsedTime spinTimer = new ElapsedTime();

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();

        // Get the blackboard pose that was set during auto or if it was not set set the starting pose to 0.
        RobotContainer.HardwareDevices.pinpoint.setPosition((Pose2D) blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();
    }

    @Override
    public void init_loop() {
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.intakeGamepadable = true;
        Status.flywheelToggle = false;
        robotContainer.spindexer.slotColor[0] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        robotContainer.spindexer.slotColor[1] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        robotContainer.spindexer.slotColor[2] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.isDrivingActive = true;
        robotContainer.start(this, true);
        robotContainer.spindexer.goToFirstIntakeSlot(); // This should likely be in robotcontainer start
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]); // This should likely be in the robotcontainer start
        robotContainer.gamepadEx1.gamepad.setLedColor(1,0,1,Gamepad.LED_DURATION_CONTINUOUS);
        List<FeedForward.TrajectoryPoint> path = robotContainer.feedForward.generatePath(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.CM, 100, 100, AngleUnit.DEGREES, 0), 0.02);
        robotContainer.addEventTelemetry("FeedForward Pose", path.get(path.size() - 1).pose.toString());
        robotContainer.addEventTelemetry("FeedForward Time", path.get(path.size() - 1).time);
        robotContainer.addEventTelemetry("FeedForward Velocity", path.get(path.size() - 1).velocity);
        spinTimer.reset();
    }

    @Override
    public void loop() {
        robotContainer.update(true);

        // Gamepad 1
        robotContainer.drivetrain.controlUpdate(); // For controller driving

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            Constants.USE_BETTER_IMU = !Constants.USE_BETTER_IMU;
        }

        if (robotContainer.gamepadEx2.triangle.wasJustPressed()) {
            Status.manualControl = !Status.manualControl;
        }

        // Auto drive to baseing position
        if (robotContainer.gamepadEx1.square.wasJustPressed()) {
            Status.isDrivingActive = false;
            robotContainer.pathPlanner.clearPoses();
            if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, -97, -84, AngleUnit.DEGREES, 180));
            } else {
                robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, -97, 84, AngleUnit.DEGREES, 180));
            }
            robotContainer.pathingUpdater.start();
        }

        if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
            Status.isDrivingActive = false;
            if (Status.alliance == Constants.Game.ALLIANCE.RED) {
                RobotContainer.HardwareDevices.pinpoint.setPosition(new Pose2D(DistanceUnit.CM, -155, 160, AngleUnit.DEGREES, 0));
            } else if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                RobotContainer.HardwareDevices.pinpoint.setPosition(new Pose2D(DistanceUnit.CM, -155, -160, AngleUnit.DEGREES, 0));
            }
            pinpointTimer.reset();
            RobotContainer.HardwareDevices.betterIMU.resetAngle();
            robotContainer.gamepadEx1.rumble(300);
            robotContainer.positionProvider.reset();
        }

        if (robotContainer.gamepadEx1.circle.wasJustPressed()) {
            Status.isDrivingActive = false;
            RobotContainer.HardwareDevices.pinpoint.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            pinpointTimer.reset();
            RobotContainer.HardwareDevices.betterIMU.resetAngle();
            robotContainer.gamepadEx1.rumble(300);
        }

        if (pinpointTimer.milliseconds() > 300 && !robotContainer.gamepadEx1.square.isPressed()) {
            Status.isDrivingActive = true;
        }

        if (robotContainer.gamepadEx1.square.isHeld()) {
            if (robotContainer.pathPlanner.driveUsingPID(0)) {
                Status.isDrivingActive = true;
                if (robotContainer.pathingUpdater != null) {
                    robotContainer.pathingUpdater.stopThread();
                    try {
                        robotContainer.pathingUpdater.join();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }

        if (robotContainer.gamepadEx1.square.wasJustReleased()) {
            Status.isDrivingActive = true;
            if (robotContainer.pathingUpdater != null) {
                robotContainer.pathingUpdater.stopThread();
                try {
                    robotContainer.pathingUpdater.join();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            robotContainer.pathingUpdater = new PathingUpdater(robotContainer);
            Status.isDrivingActive = true;
        }

        if (Status.intakeGamepadable) {
            double power = robotContainer.gamepadEx1.rightTriggerRaw() - (robotContainer.gamepadEx1.leftTriggerRaw());
            robotContainer.intake.setPower(Math.signum(power) * Math.min(Math.abs(power), Constants.Intake.TOP_SPEED));
        }

//        robotContainer.telemetry.addData("heading", RobotContainer.HardwareDevices.pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
//        currentHeading = Status.currentHeading;
//
//        if (currentHeading >= lastHeading - 3) {
//            if (!robotContainer.gamepadEx1.atRest.isHeld()) {
//                hvelocity = (Math.abs(currentHeading) - Math.abs(lastHeading)) / spinTimer.seconds();
//            }
//        } else {
//            if (!robotContainer.gamepadEx1.atRest.isHeld()) {
//                hvelocity = (currentHeading + (360 - lastHeading)) / spinTimer.seconds();
//            }
//        }
//
//        if (hvelocity > htopSpeed) {
//            htopSpeed = hvelocity;
//        }
//
//        currentDist = Math.sqrt(Math.pow(Status.currentPose.getX(DistanceUnit.CM), 2) + Math.pow(Status.currentPose.getY(DistanceUnit.CM), 2));
//        currentSpeed = (Math.abs(currentDist) - Math.abs(lastDist)) / spinTimer.seconds();
//
//        if (currentSpeed > topSpeed){
//            topSpeed = currentSpeed;
//        }
//
//        robotContainer.telemetry.addData("Max Heading Speed", htopSpeed);
//        robotContainer.telemetry.addData("Max Speed", topSpeed);
//
//        spinTimer.reset();
//        lastHeading = currentHeading;
//        lastDist = currentDist;

        Thread.yield();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
//        robotContainer.writeDataLog();
//        robotContainer.writeEventLog();

        Status.isDrivingActive = false;
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
