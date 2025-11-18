package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

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
    public static double AFTER_TAPE = 155;
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
        Status.slotColor[0] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[1] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        Status.slotColor[2] = Constants.Game.ARTIFACT_COLOR.PURPLE;
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, AFTER_TAPE, AngleUnit.DEGREES, 90));
            /*
            robotContainer.pathPlanner.addPose(BLUE_MIDPOINT);
            robotContainer.pathPlanner.addPose(BLUE_MIDDLE);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_HIGH, AFTER_TAPE, AngleUnit.DEGREES, 90));
            */
        } else {
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(6000);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_MID, -AFTER_TAPE, AngleUnit.DEGREES, -90));
        }

        robotContainer.delayedActionManager.schedule(() -> Status.turretToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.intakeToggle = false, 0);
        robotContainer.delayedActionManager.schedule(() -> Status.flywheelToggle = true, 0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootAll(), 500);
        //robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()
        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.flywheel.setTargetVelocity(robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal())),0);
        robotContainer.delayedActionManager.schedule(()-> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), 0);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 2);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false, 2);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true, 2);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false, 2);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.goToNextIntakeSlot(), 2);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.8), 3);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 3);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(-0.8), 4);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 4);

        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = true, 5);
//        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = false, 5);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = true, 5);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.goToNextTransferSlot(), 5);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.shootAll(), 5);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), 5);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal())),5);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 6);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false, 6);
//        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true, 6);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false, 6);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.goToNextIntakeSlot(), 6);


        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.8), 7);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 7);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(-0.8), 8);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 8);

        /*
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM += 4, 8);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM += 4, 8);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES += 8, 8);

        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_PID_TOLERANCE_CM -= 4, 9);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM -= 4, 9);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES -= 8, 9);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 10);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = true, 10);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = false, 10);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = true, 10);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.shootAll(), 10);
        robotContainer.delayedActionManager.schedulePose(()-> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]), 10);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal())),5);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 11);
        robotContainer.delayedActionManager.schedulePose(() -> Status.turretToggle = false, 11);
        robotContainer.delayedActionManager.schedulePose(() -> Status.intakeToggle = true, 11);
        robotContainer.delayedActionManager.schedulePose(() -> Status.flywheelToggle = false, 11);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.8), 11);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 11);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0), 12);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 12);
        */

        Status.turretToggle = true;
        Status.intakeToggle = false;
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

        if (robotContainer.beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }

//        if (Status.turretToggleButton.wasJustPressed()) {
//            robotContainer.spindexer.goToNextTransferSlot();
//        }

//        if (!Status.intakeToggle) {
////            robotContainer.turret.flywheel.setTargetVelocity(Math.min(Math.pow(Status.turretToggleButton.getHoldDuration(), Constants.Turret.FLYWHEEL_CURVE), robotContainer.limelightLogic.getRequiredFlywheelSpeed()));
//            robotContainer.turret.flywheel.setTargetVelocity(0.63);
//        } else {
//            robotContainer.turret.flywheel.setTargetVelocity(0);
//        }
        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
