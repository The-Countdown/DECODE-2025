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
    public static double TO_WALL_Y = 160;
    public static double INTAKE_BACKUP_Y = 110;
    public static double HALFWAY_HEADING = 135;
    public static double INTAKE_HEADING = 90;

    // Sleep Poses
    public static int SHOOT_TIME = 2000;
    public static int INTAKE_WAIT_TIME = 750;

    // Timeouts
    public static int GO_TO_START_TIME = 2000;
    public static int GO_TO_INTAKE_TIME = 2250;
    public static int SHIFTING_INTAKE_TIME = 1250;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
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
            () -> Constants.Pathing.LATITUDE_KP *= 1.3,
            () -> Constants.Pathing.LONGITUDE_KP *= 1.3,
            () -> robotContainer.spindexer.shootToggle(true),
            () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1,
            () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1
        );

        ActionPose shoot = new ActionPose(robotContainer,
            () -> robotContainer.intake.setPower(0.0),
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
                () -> robotContainer.spindexer.shootToggle(true)
        );

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 20, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, HALFWAY_HEADING), GO_TO_START_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 5, Status.startingPose.getY(DistanceUnit.INCH) + 20, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addSleepPose(INTAKE_WAIT_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), TO_WALL_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, INTAKE_BACKUP_Y, AngleUnit.DEGREES, INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(goToEnd);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 5, Status.startingPose.getY(DistanceUnit.INCH) + 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        } else {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 20, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 5, Status.startingPose.getY(DistanceUnit.INCH) - 20, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addSleepPose(INTAKE_WAIT_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

                robotContainer.pathPlanner.addActionPose(intake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), GO_TO_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -INTAKE_BACKUP_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM), -TO_WALL_Y, AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addActionPose(endOfIntake);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM ) + 10, -INTAKE_BACKUP_Y , AngleUnit.DEGREES, -INTAKE_HEADING), SHIFTING_INTAKE_TIME);
                robotContainer.pathPlanner.addPoseTimeout(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, -HALFWAY_HEADING), GO_TO_START_TIME);
                robotContainer.pathPlanner.addActionPose(shoot);
                robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);

            robotContainer.pathPlanner.addActionPose(goToEnd);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 5, Status.startingPose.getY(DistanceUnit.INCH) - 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
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
//        robotContainer.telemetry.addData("Intake Velocity: ", robotContainer.intake.getVelocity());
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.addData("Flywheel Target Velocity", robotContainer.turret.flywheel.targetVelocity);
        robotContainer.telemetry.addData("Flywheel At Velocity: ", robotContainer.turret.flywheel.atTargetVelocity());
//        robotContainer.telemetry.addData("Pause", robotContainer.spindexer.pause);
//        robotContainer.telemetry.addData("heading", robotContainer.headingPID.calculate());
//        robotContainer.telemetry.addData("longitude", robotContainer.longitudePID.calculate());
//        robotContainer.telemetry.addData("latitude", robotContainer.latitudePID.calculate());
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
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) + 12, Status.currentPose.getY(DistanceUnit.CM) + 12, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        } else {
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) - 12, Status.currentPose.getY(DistanceUnit.CM) - 12, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        }
        robotContainer.stop();
    }
}
