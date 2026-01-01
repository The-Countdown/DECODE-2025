package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathingUpdater;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pinpointTimer = new ElapsedTime();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
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
        Status.intakeToggle = true;
        Status.turretToggle = false;
        robotContainer.spindexer.slotColor[0] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        robotContainer.spindexer.slotColor[1] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        robotContainer.spindexer.slotColor[2] = Constants.Game.ARTIFACT_COLOR.UNKNOWN;
        Status.isDrivingActive = true;
        robotContainer.start(this, true);
        robotContainer.spindexer.goToFirstIntakeSlot(); // This should likely be in robotcontainer start
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]); // This should likely be in the robotcontainer start
    }

    @Override
    public void loop() {
        robotContainer.update(true);

        // Gamepad 1
        robotContainer.drivetrain.controlUpdate(); // For controller driving

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
            RobotContainer.HardwareDevices.pinpoint.setPosition(Status.cornerResetPose);
            pinpointTimer.reset();
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

        if (Status.intakeToggle) {
            double power = robotContainer.gamepadEx1.rightTriggerRaw() - (robotContainer.gamepadEx1.leftTriggerRaw());
            robotContainer.intake.setPower(Math.signum(power) * Math.min(Math.abs(power), Constants.Intake.TOP_SPEED));
        } else {
            robotContainer.intake.setPower(Constants.Intake.REVERSE_TOP_SPEED);
        }

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
