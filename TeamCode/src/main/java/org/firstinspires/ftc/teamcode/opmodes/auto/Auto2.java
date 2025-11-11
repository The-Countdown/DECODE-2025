package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;

@Autonomous(name="Auto2", group="Robot")
@Config
public class Auto2 extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;

    //102.22
    //91.44
    //30.48
    public static double BEFORE_TAPE = 84;
    public static double AFTER_TAPE = 132;
    public static double TAPE_LOW = -91.5;
    public static double TAPE_MID = -34.5;
    public static double TAPE_HIGH = 26.5;
    public static double MIDPOINT = 18;
    public static double MIDDLE = 20;

    public static Pose2D
            RED_MIDDLE = new Pose2D(DistanceUnit.INCH, MIDDLE, -MIDDLE, AngleUnit.DEGREES, -135),
            RED_MIDPOINT = new Pose2D(DistanceUnit.INCH, 0, -MIDPOINT, AngleUnit.DEGREES, -112.5),

            BLUE_MIDDLE = new Pose2D(DistanceUnit.INCH, MIDDLE, MIDDLE, AngleUnit.DEGREES, 135),
            BLUE_MIDPOINT = new Pose2D(DistanceUnit.INCH, 0, MIDPOINT, AngleUnit.DEGREES, 112.5);

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        blackboard.put("pose", Status.currentPose);
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        Status.intakeToggle = true;
        Status.turretToggle = false;
        robotContainer.start(this, true);
        // This is important do not remove it, we do not know why it is here. (Cole, Elliot)
        robotContainer.localizationUpdater = new LocalizationUpdater(robotContainer);
        robotContainer.localizationUpdater.start();

        if (Status.wentBackToStart) {
            Status.startingPose = (Pose2D) blackboard.getOrDefault("pose", Status.startingPose);
        }
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(3000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(BLUE_MIDPOINT);
            robotContainer.pathPlanner.addPose(BLUE_MIDDLE);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, AFTER_TAPE, AngleUnit.DEGREES, 90));
        } else {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(3000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(RED_MIDPOINT);
            robotContainer.pathPlanner.addPose(RED_MIDDLE);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, -AFTER_TAPE, AngleUnit.DEGREES, -90));
        }

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0.1), 1);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 1);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0), 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 2);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.shootNextBall(), 3);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0.63), 3);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 4);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false, 5);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true, 5);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false, 5);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0.3), 5);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 5);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0), 6);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 6);

        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM += 4, 8);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM += 4, 8);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES += 8, 8);

        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM -= 4, 9);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM -= 4, 9);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES -= 8, 9);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0.3), 9);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 9);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0), 10);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 10);

        Status.turretToggle = true;
        Status.intakeToggle = false;

//        robotContainer.pathPlanner.setTarget(1);
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setVelocity(0.8), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(2));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(2), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(2));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setVelocity(0), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.flywheel.setTargetVelocity(0.5), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(3), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(4));
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus();
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath();
        robotContainer.beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());
        Status.turretToggleButton.update(Status.turretToggle);
        robotContainer.telemetry.addData("Flywheel Toggle: ", Status.flywheelToggle);
        robotContainer.telemetry.addData("Intake Toggle: ", Status.intakeToggle);
        robotContainer.telemetry.addData("Turret Toggle: ", Status.turretToggle);
        robotContainer.telemetry.addData("Intake Velocity: ", robotContainer.intake.getVelocity());
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.update();


        double spindexerError = Math.abs(robotContainer.spindexer.pdf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - spindexerError) > 50) {
            spindexAccel.reset();
        }

        if (spindexerError > 2) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pdf.calculate() * spindexAccel.seconds(), 1));
            } else {
                robotContainer.spindexer.setPower(robotContainer.spindexer.pdf.calculate());
            }
        } else {
            robotContainer.spindexer.setPower(0);
        }
        lastError = spindexerError;

        if (robotContainer.beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        if (Status.turretToggleButton.wasJustPressed()) {
            robotContainer.spindexer.goToNextTransferSlot();
        }

//        if (!Status.intakeToggle) {
////            robotContainer.turret.flywheel.setTargetVelocity(Math.min(Math.pow(Status.turretToggleButton.getHoldDuration(), Constants.Turret.FLYWHEEL_CURVE), robotContainer.limelightLogic.getRequiredFlywheelSpeed()));
//            robotContainer.turret.flywheel.setTargetVelocity(0.63);
//        } else {
//            robotContainer.turret.flywheel.setTargetVelocity(0);
//        }
//        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
