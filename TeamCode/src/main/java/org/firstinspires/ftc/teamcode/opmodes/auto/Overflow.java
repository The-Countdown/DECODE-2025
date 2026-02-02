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

@Autonomous(name="Overflow", group="Robot")
@Config
public class Overflow extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // Positions
    public static double
        SHOOTING_FAR_Y = 50,
        TO_WALL_Y = 157,
        INTAKE_BACKUP_Y = 120,
        SHOOTING_FAR_X = -145,
        HUMAN_PLAYER_STATION_X = -161;

    // Headings
    public static double
        INTAKE_HEADING = 90,
        HALFWAY_HEADING = 135;

    // Timeouts
    public static int
        GO_TO_INTAKE_TIME = 1250,
        SHIFTING_INTAKE_TIME = 1000,
        SHORT_SHIFTING_INTAKE_TIME = 750,
        GO_TO_SHOOT_TIME = 1750,
        GO_TO_START_TIME = 1250;

    // Sleep Poses
    public static int
            SHOOT_TIME = 1300;

    // Poses
    public static Pose2D
        // Shooting Poses
            RED_SHOOTING_FAR_TURNED = new Pose2D(DistanceUnit.CM, SHOOTING_FAR_X, -SHOOTING_FAR_Y, AngleUnit.DEGREES, -HALFWAY_HEADING),
            BLUE_SHOOTING_FAR_TURNED = new Pose2D(DistanceUnit.CM, SHOOTING_FAR_X, SHOOTING_FAR_Y, AngleUnit.DEGREES, HALFWAY_HEADING),

        // Human Player Station Poses
            RED_HUMAN_PLAYER_STATION_WALL = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_HUMAN_PLAYER_STATION_BACKUP = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING),

            BLUE_HUMAN_PLAYER_STATION_WALL = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_HUMAN_PLAYER_STATION_BACKUP = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // In-Between Poses
            RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X + 10, -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION = new Pose2D(DistanceUnit.CM, HUMAN_PLAYER_STATION_X + 10, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // End Poses
            RED_END_FAR = new Pose2D(DistanceUnit.CM,  -110, -56, AngleUnit.DEGREES, 180),
            BLUE_END_FAR = new Pose2D(DistanceUnit.CM, -110,  56, AngleUnit.DEGREES, 180);
    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        RobotContainer.HardwareDevices.betterIMU.setAngleOffset(180);
        Status.waitToShoot = true;
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
            () -> Constants.Pathing.LATITUDE_KP *= 1.5, //To speed up only change the first two of KP change
            () -> Constants.Pathing.LONGITUDE_KP *= 1.5,
            () -> Constants.Pathing.LATITUDE_KP *= 1.5,
            () -> Constants.Pathing.LONGITUDE_KP *= 1.5,
            () -> robotContainer.spindexer.shootToggle(true),
            () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1,
            () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1
        );

        ActionPose shoot = new ActionPose(robotContainer,
            () -> robotContainer.intake.setPower(0.0),
            () -> Constants.Pathing.LATITUDE_KP /= 1.5,
            () -> Constants.Pathing.LONGITUDE_KP /= 1.5,
            () -> robotContainer.spindexer.shootAll(false)
        );

        ActionPose goToEnd = new ActionPose(robotContainer,
            () -> robotContainer.intake.setPower(0.0),
            () -> robotContainer.spindexer.shootToggle(false)
        );

        ActionPose intake = new ActionPose(robotContainer,
            () -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED),
            () -> robotContainer.spindexer.shootToggle(false)
        );

        ActionPose endOfIntake = new ActionPose(robotContainer,
            () -> robotContainer.intake.function3(),
            () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED), 200),
            () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 400),
            () -> Constants.Pathing.LATITUDE_KP *= 1.5,
            () -> Constants.Pathing.LONGITUDE_KP *= 1.5,
            () -> robotContainer.spindexer.shootToggle(true)
        );

        ActionPose speedUp = new ActionPose(robotContainer,
            () -> Constants.Pathing.LATITUDE_KP *= 1.5,
            () -> Constants.Pathing.LONGITUDE_KP *= 1.5
        );

        ActionPose slowDown = new ActionPose(robotContainer,
                () -> Constants.Pathing.LATITUDE_KP /= 1.5,
                () -> Constants.Pathing.LONGITUDE_KP /= 1.5
        );

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR_TURNED, GO_TO_START_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(goToEnd);
            robotContainer.pathPlanner.addPose(BLUE_END_FAR);
        } else {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR_TURNED, GO_TO_START_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, GO_TO_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(speedUp);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_BACKUP, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(slowDown);
            robotContainer.pathPlanner.addPoseTimeout(RED_HUMAN_PLAYER_STATION_WALL, SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_MIDDLE_OF_FAR_SHOOT_AND_HUMAN_PLAYER_STATION, SHORT_SHIFTING_INTAKE_TIME);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_FAR_TURNED, GO_TO_SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(goToEnd);
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
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.addData("Flywheel Target Velocity", robotContainer.turret.flywheel.targetVelocity);
        robotContainer.telemetry.addData("Flywheel At Velocity: ", robotContainer.turret.flywheel.atTargetVelocity());
        robotContainer.telemetry.addData("drive target", Status.targetPose);
        robotContainer.telemetry.addData("Current Pose", Status.currentPose);
        robotContainer.telemetry.addData("At target", PoseMath.isAtPos());
        robotContainer.telemetry.addData("Timeout", robotContainer.pathPlanner.timeoutCheck);
        robotContainer.telemetry.addData("spindex colors", robotContainer.spindexer.slotColor[0].toString(), robotContainer.spindexer.slotColor[1].toString(), robotContainer.spindexer.slotColor[2].toString());
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
