package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.Arrays;

@Autonomous(name="ThreeBallAuto", group="Robot")
@Config
public class ThreeBallAuto extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // 102.22
    // 91.44
    // 30.48
    public static double BEFORE_TAPE = 84;
    public static double AFTER_TAPE = 159;
    public static double TAPE_LOW = -91.5;
    public static double TAPE_MID = -34.5;
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

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH), Status.startingPose.getY(DistanceUnit.INCH) + 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(15000);
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH), Status.startingPose.getY(DistanceUnit.INCH) + 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        } else {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH), Status.startingPose.getY(DistanceUnit.INCH) - 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(15000);
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH), Status.startingPose.getY(DistanceUnit.INCH)  - 20, AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        }
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        robotContainer.start(this, false);
        Status.intakeToggle = true;
        Status.turretToggle = false;
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);

        if (Status.wentBackToStart) {
            Status.startingPose = (Pose2D) blackboard.getOrDefault("pose", Status.startingPose);
        }
        robotContainer.spindexer.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        robotContainer.spindexer.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        robotContainer.spindexer.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;

        //Start
        robotContainer.delayedActionManager.incrementPoseOffset(3);
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.pause(), Constants.Turret.FLYWHEEL_SPINUP_MS));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(false), Constants.Turret.FLYWHEEL_SPINUP_MS));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(), 5000 + Constants.Spindexer.FULL_EMPTY_SPINTIME));
        robotContainer.delayedActionManager.schedulePose(()-> Constants.Pathing.LATITUDE_KP *= 1.5);
        robotContainer.delayedActionManager.schedulePose(()-> Constants.Pathing.LONGITUDE_KP *= 1.5);

        //Going to intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.positionProvider.update(false);
//        robotContainer.limelightLogic.update();
        robotContainer.delayedActionManager.update();
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath(pathTimer);
        robotContainer.beamBreakToggleButton.update(RobotContainer.HardwareDevices.beamBreak.isPressed());
        Status.turretToggleButton.update(Status.turretToggle);
        robotContainer.CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        robotContainer.DELTA_TIME_MS = robotContainer.CURRENT_LOOP_TIME_MS - robotContainer.PREV_LOOP_TIME_MS;
        robotContainer.telemetry.addData("Flywheel Toggle: ", Status.flywheelToggle);
        robotContainer.telemetry.addData("Intake Toggle: ", Status.intakeToggle);
        robotContainer.telemetry.addData("Turret Toggle: ", Status.turretToggle);
        robotContainer.telemetry.addData("Intake Velocity: ", robotContainer.intake.getVelocity());
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.addData("Pause", robotContainer.spindexer.pause);
        robotContainer.telemetry.addData("heading", robotContainer.headingPID.calculate());
        robotContainer.telemetry.addData("longitude", robotContainer.longitudePID.calculate());
        robotContainer.telemetry.addData("latitude", robotContainer.latitudePID.calculate());
        robotContainer.telemetry.addData("drive target", Status.targetPose);
        robotContainer.telemetry.update();
        robotContainer.turret.update(false);
        robotContainer.spindexer.update(false);
        blackboard.put("pose", Status.currentPose);
    }
    //3.4 -6.8

    @Override
    public void stop() {
        Constants.Pathing.LONGITUDE_KP = 0.014;
        Constants.Pathing.LATITUDE_KP = 0.014;
        Constants.Pathing.LATITUDE_PID_TOLERANCE_CM = 1;
        Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM = 1;
        robotContainer.delayedActionManager.cancelAll();
        if(Status.alliance == Constants.Game.ALLIANCE.RED) {
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) + 12, Status.currentPose.getY(DistanceUnit.CM) + 12, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        } else {
            blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM) - 12, Status.currentPose.getY(DistanceUnit.CM) - 12, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        }
        robotContainer.stop();
    }
}
