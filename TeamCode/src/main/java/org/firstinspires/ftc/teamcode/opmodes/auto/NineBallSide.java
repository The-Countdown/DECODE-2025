package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PoseMath;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses.ActionPose;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous(name="NineBallSide", group="Robot")
@Config
public class NineBallSide extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();
    public static double BEFORE_TAPE = 84;
    public static double AFTER_TAPE = 162;
    public static double TAPE_LOW = -97;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        blackboard.put("pose", Status.currentPose);

        Status.startingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, Constants.Robot.STARTING_Y, AngleUnit.DEGREES, Constants.Robot.STARTING_HEADING) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, -Constants.Robot.STARTING_Y, AngleUnit.DEGREES, Constants.Robot.STARTING_HEADING) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Status.targetPose = Status.startingPose;
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);

        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        //Actions
        ActionPose start = new ActionPose(robotContainer,
                () -> Constants.Pathing.LATITUDE_KP *= 1.2,
                () -> Constants.Pathing.LONGITUDE_KP *= 1.2,
                () -> robotContainer.spindexer.shootToggle(true),
                () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1,
                () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1
        );

        ActionPose shoot = new ActionPose(robotContainer,
                () -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED),
                () -> robotContainer.spindexer.shootAll(false),
                () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM /= 1.5,
                () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM /= 1.5
        );

        ActionPose goToIntake = new ActionPose(robotContainer,
                () -> robotContainer.intake.setPower(0.0),
                () -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES /= 2,
                () -> robotContainer.spindexer.shootToggle(false)
        );

        ActionPose intake = new ActionPose(robotContainer,
                () -> Constants.Pathing.LONGITUDE_KP /= 2,
                () -> Constants.Pathing.LATITUDE_KP /= 2,
                () -> Constants.Pathing.HEADING_KP /= 2,
                () -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED)
        );

        ActionPose sideGoToAndIntake = new ActionPose(robotContainer,
                () -> robotContainer.spindexer.shootToggle(false),
                () -> robotContainer.delayedActionManager.schedule(() -> Constants.Pathing.LATITUDE_KP /= 1.25, 2500),
                () -> robotContainer.delayedActionManager.schedule(() -> Constants.Pathing.LATITUDE_KP /= 1.25,  2500),
                () -> robotContainer.delayedActionManager.schedule(() -> Constants.Pathing.LATITUDE_KP *= 1.25, 3250),
                () -> robotContainer.delayedActionManager.schedule(() -> Constants.Pathing.LATITUDE_KP *= 1.25,  3250),
                () -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED)
        );

        ActionPose endOfSideIntake = new ActionPose(robotContainer,
                () -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED),
                () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 100),
                () -> robotContainer.spindexer.shootToggle(true),
                () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1.5,
                () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1.5
        );

        ActionPose endOfIntake = new ActionPose(robotContainer,
                () -> Constants.Pathing.LONGITUDE_KP *= 2,
                () -> Constants.Pathing.LATITUDE_KP *= 2,
                () -> Constants.Pathing.HEADING_KP *= 2,
                () -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED),
                () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 100),
                () -> robotContainer.spindexer.shootToggle(true),
                () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1.5,
                () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1.5,
                () -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES *= 2
        );

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addSleepPose(Constants.Turret.FLYWHEEL_SPINUP_MS);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(3000);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, TAPE_LOW, BEFORE_TAPE, AngleUnit.DEGREES, 90), 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, TAPE_LOW, AFTER_TAPE, AngleUnit.DEGREES, 90), 4000);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(2000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 30, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 1250);
            robotContainer.pathPlanner.addActionPose(sideGoToAndIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 1, AFTER_TAPE, AngleUnit.DEGREES, 90), 2750);
            robotContainer.pathPlanner.addSleepPose(1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 1, AFTER_TAPE - 25, AngleUnit.DEGREES, 90), 1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 3, AFTER_TAPE, AngleUnit.DEGREES, 90), 1000);
            robotContainer.pathPlanner.addSleepPose(1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, AFTER_TAPE - 5, AngleUnit.DEGREES, 90), 1000);
            robotContainer.pathPlanner.addActionPose(endOfSideIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(2000);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH) + 5, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        } else {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addSleepPose(Constants.Turret.FLYWHEEL_SPINUP_MS);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(3000);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, TAPE_LOW, -BEFORE_TAPE, AngleUnit.DEGREES, -90), 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, TAPE_LOW, -AFTER_TAPE, AngleUnit.DEGREES, -90), 4000);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(2000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 30, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 1250);
            robotContainer.pathPlanner.addActionPose(sideGoToAndIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 1, -AFTER_TAPE, AngleUnit.DEGREES, -90), 2750);
            robotContainer.pathPlanner.addSleepPose(1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 1, -AFTER_TAPE + 25, AngleUnit.DEGREES, -90), 1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) - 3, -AFTER_TAPE, AngleUnit.DEGREES, -90), 1000);
            robotContainer.pathPlanner.addSleepPose(1000);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, AFTER_TAPE +  5, AngleUnit.DEGREES, 90), 1000);
            robotContainer.pathPlanner.addActionPose(endOfSideIntake);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)), 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(2000);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH) - 5, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        }
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        robotContainer.start(this, false);
        robotContainer.spindexer.shootToggle(true);
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);

        if (Status.wentBackToStart) {
            Status.startingPose = (Pose2D) blackboard.getOrDefault("pose", Status.startingPose);
        }
        robotContainer.spindexer.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        robotContainer.spindexer.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        robotContainer.spindexer.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        robotContainer.pathingUpdater.timer.reset();
    }

    @Override
    public void loop() {
        robotContainer.update(false);
        robotContainer.allIndicatorLights.lightsUpdate();
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath(pathTimer);
        Status.turretToggleButton.update(Status.flywheelToggle);

        robotContainer.telemetry.addData("Flywheel Toggle: ", Status.flywheelToggle);
        robotContainer.telemetry.addData("Intake Toggle: ", Status.intakeGamepadable);
        robotContainer.telemetry.addData("Intake Velocity: ", robotContainer.intake.getVelocity());
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.addData("Pause", robotContainer.spindexer.pause);
        robotContainer.telemetry.addData("heading", robotContainer.headingPID.calculate());
        robotContainer.telemetry.addData("longitude", robotContainer.longitudePID.calculate());
        robotContainer.telemetry.addData("latitude", robotContainer.latitudePID.calculate());
        robotContainer.telemetry.addData("drive target", Status.targetPose);
        robotContainer.telemetry.addData("Current Pose", Status.currentPose);
        robotContainer.telemetry.addData("At target", PoseMath.isAtPos());
        robotContainer.telemetry.addData("Timeout", robotContainer.pathPlanner.timeoutCheck);
        robotContainer.telemetry.update();

        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        robotContainer.delayedActionManager.cancelAll();
        robotContainer.telemetry.addData("Stopped", true);
        telemetry.update();

        Constants.Pathing.LATITUDE_PID_TOLERANCE_CM = 2;
        Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM = 2;
        Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES = 10;
        Constants.Pathing.LONGITUDE_KP = 0.009;
        Constants.Pathing.LATITUDE_KP = 0.009;
        Constants.Pathing.HEADING_KP = 0.01;

        if(Status.alliance == Constants.Game.ALLIANCE.RED) {
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) + Constants.Pathing.X_OFFSET_FOR_AUTO, Status.currentPose.getY(DistanceUnit.CM) + Constants.Pathing.Y_OFFSET_FOR_AUTO, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        } else {
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) - Constants.Pathing.X_OFFSET_FOR_AUTO, Status.currentPose.getY(DistanceUnit.CM) - Constants.Pathing.Y_OFFSET_FOR_AUTO, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        }
        robotContainer.stop();
    }
}
