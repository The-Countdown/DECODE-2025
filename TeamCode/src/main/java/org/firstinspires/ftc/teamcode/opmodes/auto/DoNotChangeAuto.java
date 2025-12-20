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

@Autonomous(name="BasicThreeBallAuto", group="Robot")
@Config
public class DoNotChangeAuto extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();

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
        Status.flywheelToggle = false;
        robotContainer.start(this, false);
        // This is important do not remove it, we do not know why it is here. (Cole, Elliot)

        if (Status.wentBackToStart) {
            Status.startingPose = (Pose2D) blackboard.getOrDefault("pose", Status.startingPose);
        }
        Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH)+20, Status.startingPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
        } else {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, Status.startingPose.getX(DistanceUnit.INCH)+20, Status.startingPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, Status.startingPose.getHeading(AngleUnit.DEGREES)));
            robotContainer.pathPlanner.updatePathTimesAmount();
        }
        robotContainer.pathPlanner.updatePathTimesAmount();

        robotContainer.delayedActionManager.schedule(() -> pathTimer.reset(), 0);
        robotContainer.delayedActionManager.schedule(() -> Status.flywheelToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.intakeToggle = false, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.turretToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), 0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.pause(), Constants.Turret.FLYWHEEL_SPINUP_MS);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(false), Constants.Turret.FLYWHEEL_SPINUP_MS);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.unpause(), Constants.Turret.FLYWHEEL_SPINUP_MS + 5000);

        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());

        robotContainer.delayedActionManager.incrementPoseOffset();
        robotContainer.delayedActionManager.schedulePose(() -> pathTimer.reset());
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.limelightLogic.update();
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus(pathTimer);
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
        robotContainer.telemetry.update();
        robotContainer.turret.update(false);
        robotContainer.spindexer.update(false);
        robotContainer.positionProvider.update(false);
        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        robotContainer.delayedActionManager.cancelAll();
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}