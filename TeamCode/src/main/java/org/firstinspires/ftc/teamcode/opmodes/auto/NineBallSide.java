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
    // Positions
    public static double
            SHOOTING_FAR_Y = 43,
            BEFORE_TAPE_Y = 84,
            AFTER_TAPE_Y = 162,
            TO_WALL_Y = 157,
            INTAKE_BACKUP_Y = 110,
            SHOOTING_FAR_X = -151,
            HUMAN_PLAYER_STATION_X = -161,
            TAPE_LOW_X = -97;

    // Headings
    public static double
            INTAKE_HEADING = 90,
            HALFWAY_HEADING = 135;

    // Timeouts
    public static int
            INTAKE_TIME = 4500,
            GO_TO_INTAKE_TIME = 2000,
            SHIFTING_INTAKE_TIME = 1000,
            SHORT_SHIFTING_INTAKE_TIME = 750;

    // Sleep Poses
    public static int
            SHOOT_TIME = 2000;

    // Poses
    public static Pose2D
        // Shooting Poses
            RED_SHOOTING_FAR = new Pose2D(DistanceUnit.CM, SHOOTING_FAR_X, -SHOOTING_FAR_Y, AngleUnit.DEGREES, 180),
            BLUE_SHOOTING_FAR = new Pose2D(DistanceUnit.CM, SHOOTING_FAR_X, SHOOTING_FAR_Y, AngleUnit.DEGREES, 180),

        // Tape Poses
            // Red
            RED_BEFORE_LOW_TAPE = new Pose2D(DistanceUnit.CM,TAPE_LOW_X, -BEFORE_TAPE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_AFTER_LOW_TAPE = new Pose2D(DistanceUnit.CM,TAPE_LOW_X, -AFTER_TAPE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),

            // Blue
            BLUE_BEFORE_LOW_TAPE = new Pose2D(DistanceUnit.CM,TAPE_LOW_X, BEFORE_TAPE_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_AFTER_LOW_TAPE = new Pose2D(DistanceUnit.CM,TAPE_LOW_X, AFTER_TAPE_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // Human Player Station Poses
            RED_HUMAN_PLAYER_STATION_WALL = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_HUMAN_PLAYER_STATION_BACKUP = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING),

            BLUE_HUMAN_PLAYER_STATION_WALL = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_HUMAN_PLAYER_STATION_BACKUP = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // In-Between Poses
            RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X + 15, -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X + 15, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // End Poses
            RED_END_FAR = new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH) - 5, AngleUnit.DEGREES, 180),
            BLUE_END_FAR = new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH) + 5, AngleUnit.DEGREES, 180);

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        RobotContainer.HardwareDevices.betterIMU.setAngleOffset(180);
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
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_BEFORE_LOW_TAPE, 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_AFTER_LOW_TAPE, INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR, 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR, 2000);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPose(BLUE_END_FAR);
        } else {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addSleepPose(Constants.Turret.FLYWHEEL_SPINUP_MS);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_BEFORE_LOW_TAPE, 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_AFTER_LOW_TAPE, INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR, 2250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR, 2000);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPose(RED_END_FAR);
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
