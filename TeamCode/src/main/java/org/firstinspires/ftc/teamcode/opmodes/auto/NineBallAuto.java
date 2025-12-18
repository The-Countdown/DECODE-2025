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

@Autonomous(name="NineBallAuto", group="Robot")
@Config
public class NineBallAuto extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // 102.22
    // 91.44
    // 30.48
    public static double BEFORE_TAPE = 84;
    public static double AFTER_TAPE = 162;
    public static double TAPE_LOW = -96.5;
    public static double TAPE_MID = -39.5;
    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        blackboard.put("pose", Status.currentPose);
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        Status.startingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.startingX, Constants.Robot.startingY, AngleUnit.DEGREES, Constants.Robot.startingHeading) :
                Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.startingX, -Constants.Robot.startingY, AngleUnit.DEGREES, Constants.Robot.startingHeading) :
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(3600);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(400);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(2000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(400);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(2000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        } else {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(3600);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(400);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(2000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(400);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Status.startingPose.getX(DistanceUnit.CM) + 10, Status.startingPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.addPose(2000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH) + 20, Status.startingPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        }
//        robotContainer.pathPlanner.updatePathTimesAmount();
//        robotContainer.pathPlanner.updatePathTimes();

        robotContainer.telemetry.addLine(Arrays.toString(robotContainer.pathPlanner.estimatedPathTimes));
        robotContainer.telemetry.update();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        robotContainer.start(this, false);
        Status.isDrivingActive = false;
        Status.intakeToggle = true;
        Status.turretToggle = false;
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);

        if (Status.wentBackToStart) {
            Status.startingPose = (Pose2D) blackboard.getOrDefault("pose", Status.startingPose);
        }
        Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);

        //Start
        robotContainer.delayedActionManager.schedule(() -> pathTimer.reset(), 0);
        robotContainer.delayedActionManager.schedule(() -> Status.flywheelToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.intakeToggle = false, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.turretToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.pause(), Constants.Turret.FLYWHEEL_SPINUP_MS);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(false), Constants.Turret.FLYWHEEL_SPINUP_MS);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(), Constants.Turret.FLYWHEEL_SPINUP_MS + Constants.Spindexer.FULL_EMPTY_SPINTIME);

        //Start of sleepPose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM /= 2);

        //Going to intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);

        //At intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED));

        //Start of sleepPose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());

        //End of intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 15,0);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 15,0);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 100));
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = true);

        //Starting pose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.pause());
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(false),  300));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(),  Constants.Spindexer.FULL_EMPTY_SPINTIME));

        //Going to Intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM /= 15);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM /= 15);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.0));
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);

        //At intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED));

        //Start of sleepPose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());

        //End of Intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 15);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 15);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 100));
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = true);

        //Starting pose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.pause());
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.shootAll(false));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(),  Constants.Spindexer.FULL_EMPTY_SPINTIME));

        //Going to intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM /= 15);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM /= 15);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.0));
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);

        //Beginning of intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP /= 2);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(Constants.Intake.BEST_INTAKE_SPEED));

        //Start of sleepPose
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());

        //End of Intake
        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_KP *= 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM *= 15);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM *= 15);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(-Constants.Intake.BEST_INTAKE_SPEED));
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.0), 100));
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.limelightLogic.update();
        robotContainer.delayedActionManager.update();
//        robotContainer.pathPlanner.updatePathStatus(pathTimer);
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath();
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
        robotContainer.positionProvider.update(false);
        blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM), Status.currentPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
    }
    //3.4 -6.8

    @Override
    public void stop() {
        Constants.Pathing.LONGITUDE_KP = 0.014;
        Constants.Pathing.LATITUDE_KP = 0.014;
        Constants.Pathing.LATITUDE_PID_TOLERANCE_CM = 1;
        Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM = 1;
        robotContainer.delayedActionManager.cancelAll();
        blackboard.put("pose", new Pose2D(DistanceUnit.CM, Status.currentPose.getX(DistanceUnit.CM), Status.currentPose.getY(DistanceUnit.CM)+9, AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
        robotContainer.stop();
    }
}
