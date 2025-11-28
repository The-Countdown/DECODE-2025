package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous(name="ThreeBallAuto", group="Robot")
@Config
public class DoNotChangeAuto extends OpMode {
    private RobotContainer robotContainer;

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
        robotContainer.start(this, true);
        // This is important do not remove it, we do not know why it is here. (Cole, Elliot)
        robotContainer.localizationUpdater = new LocalizationUpdater(robotContainer);
        robotContainer.localizationUpdater.start();

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
        }

        robotContainer.delayedActionManager.schedule(() -> Status.flywheelToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.intakeToggle = false, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.turretToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(false), 800);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), 0);

        robotContainer.delayedActionManager.incrementPoseOffset(2);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false);
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus();
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath();
        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), 0);
        robotContainer.telemetry.addData("Flywheel Toggle: ", Status.flywheelToggle);
        robotContainer.telemetry.addData("Intake Toggle: ", Status.intakeToggle);
        robotContainer.telemetry.addData("Turret Toggle: ", Status.turretToggle);
        robotContainer.telemetry.addData("Intake Velocity: ", robotContainer.intake.getVelocity());
        robotContainer.telemetry.addData("Flywheel Velocity: ", RobotContainer.HardwareDevices.flyWheelMotorMaster.getVelocity());
        robotContainer.telemetry.update();

        if (!Status.intakeToggle) {
            robotContainer.intake.setPower(Constants.Intake.REVERSE_TOP_SPEED);
        } else {
            robotContainer.intake.setPower(0);
        }
        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
