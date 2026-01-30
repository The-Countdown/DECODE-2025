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

@Autonomous(name="NineBallCloseGate", group="Robot")
@Config
public class NineBallCloseGate extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // Positions
    public static double
            BEFORE_TAPE_Y = 80,
            AFTER_TAPE_Y = 157,
            MIDPOINT_Y = 18,
            TAPE_LOW_X = -97,
            TAPE_MID_X = -37,
            TAPE_HIGH_X = 25,
            GATE_X = 0,
            GATE_Y = 145,
            MIDDLE_XY = 20;

    // Headings
    public static double
            INTAKE_HEADING = 90,
            HALFWAY_HEADING = 150;

    // Sleep Poses
    public static int
            SHOOT_TIME = 1300;

    // Poses
    public static Pose2D
        // Shooting Poses
            RED_SHOOTING_CLOSE = new Pose2D(DistanceUnit.INCH, MIDDLE_XY, -MIDDLE_XY, AngleUnit.DEGREES, -HALFWAY_HEADING),
            BLUE_SHOOTING_CLOSE = new Pose2D(DistanceUnit.INCH, MIDDLE_XY, MIDDLE_XY, AngleUnit.DEGREES, HALFWAY_HEADING),

        // Tape Poses
            // Red
            RED_BEFORE_MID_TAPE = new Pose2D(DistanceUnit.CM,TAPE_MID_X, -BEFORE_TAPE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_AFTER_MID_TAPE = new Pose2D(DistanceUnit.CM,TAPE_MID_X, -AFTER_TAPE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_BEFORE_HIGH_TAPE = new Pose2D(DistanceUnit.CM,TAPE_HIGH_X, -BEFORE_TAPE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_AFTER_HIGH_TAPE = new Pose2D(DistanceUnit.CM,TAPE_HIGH_X, -AFTER_TAPE_Y + 10, AngleUnit.DEGREES, -INTAKE_HEADING),

            // Blue
            BLUE_BEFORE_MID_TAPE = new Pose2D(DistanceUnit.CM,TAPE_MID_X, BEFORE_TAPE_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_AFTER_MID_TAPE = new Pose2D(DistanceUnit.CM,TAPE_MID_X, AFTER_TAPE_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_BEFORE_HIGH_TAPE = new Pose2D(DistanceUnit.CM,TAPE_HIGH_X, BEFORE_TAPE_Y, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_AFTER_HIGH_TAPE = new Pose2D(DistanceUnit.CM,TAPE_HIGH_X, AFTER_TAPE_Y - 10, AngleUnit.DEGREES, INTAKE_HEADING),
        // Gate Poses
            RED_GATE_BEFORE = new Pose2D(DistanceUnit.CM, TAPE_MID_X, -130, AngleUnit.DEGREES, -INTAKE_HEADING),
            RED_GATE_AFTER = new Pose2D(DistanceUnit.CM, GATE_X, -GATE_Y, AngleUnit.DEGREES, -INTAKE_HEADING),

            BLUE_GATE_BEFORE = new Pose2D(DistanceUnit.CM, TAPE_MID_X, 130, AngleUnit.DEGREES, INTAKE_HEADING),
            BLUE_GATE_AFTER = new Pose2D(DistanceUnit.CM, GATE_X, GATE_Y, AngleUnit.DEGREES, INTAKE_HEADING),

        // End Poses
            RED_END_CLOSE = new Pose2D(DistanceUnit.INCH, 0, -MIDPOINT_Y, AngleUnit.DEGREES, -112.5),
            BLUE_END_CLOSE = new Pose2D(DistanceUnit.INCH, 0, MIDPOINT_Y, AngleUnit.DEGREES, 112.5);
    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        blackboard.put("pose", Status.currentPose);

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE){
            Status.startingPose = new Pose2D(DistanceUnit.INCH, 48, 52, AngleUnit.DEGREES, 180);
        } else {
            Status.startingPose = new Pose2D(DistanceUnit.INCH, 48, -52, AngleUnit.DEGREES, 180);
        }
        RobotContainer.HardwareDevices.betterIMU.setAngleOffset(Status.startingPose.getHeading(AngleUnit.DEGREES));


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
            () -> robotContainer.spindexer.pause(),
            () -> robotContainer.spindexer.shootAll(false),
            () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(),  Constants.Spindexer.FULL_EMPTY_SPINTIME),
            () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM /= 1.5,
            () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM /= 1.5
        );

        ActionPose goToEnd = new ActionPose(robotContainer,
            () -> robotContainer.intake.setPower(0.0),
            () -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES /= 2,
            () -> robotContainer.spindexer.shootToggle(false)
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

        ActionPose endOfIntake = new ActionPose(robotContainer,
                () -> Constants.Pathing.LONGITUDE_KP *= 2,
                () -> Constants.Pathing.LATITUDE_KP *= 2,
                () -> Constants.Pathing.HEADING_KP *= 2,
                () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED), 500),
                () -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 700),
                () -> robotContainer.spindexer.shootToggle(true),
                () -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 1.5,
                () -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 1.5,
                () -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES *= 2
        );

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_CLOSE, 1250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_BEFORE_HIGH_TAPE, 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_AFTER_HIGH_TAPE, 2000);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_GATE_BEFORE, 750);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_GATE_AFTER, 1250);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_CLOSE, 1250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_BEFORE_MID_TAPE, 1500);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_AFTER_MID_TAPE, 2500);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(BLUE_SHOOTING_CLOSE, 2500);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToEnd);
            robotContainer.pathPlanner.addPose(BLUE_END_CLOSE);
        } else {
            robotContainer.pathPlanner.addActionPose(start);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_CLOSE, 1250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_BEFORE_HIGH_TAPE, 1750);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_AFTER_HIGH_TAPE, 2000);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_GATE_BEFORE, 750);
            robotContainer.pathPlanner.addPoseTimeout(RED_GATE_AFTER, 1250);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_CLOSE, 1250);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_BEFORE_MID_TAPE, 1500);
            robotContainer.pathPlanner.addActionPose(intake);
            robotContainer.pathPlanner.addPoseTimeout(RED_AFTER_MID_TAPE, 2500);
            robotContainer.pathPlanner.addActionPose(endOfIntake);
            robotContainer.pathPlanner.addPoseTimeout(RED_SHOOTING_CLOSE, 2500);
            robotContainer.pathPlanner.addActionPose(shoot);
            robotContainer.pathPlanner.addSleepPose(SHOOT_TIME);
            robotContainer.pathPlanner.addActionPose(goToEnd);
            robotContainer.pathPlanner.addPose(RED_END_CLOSE);

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
