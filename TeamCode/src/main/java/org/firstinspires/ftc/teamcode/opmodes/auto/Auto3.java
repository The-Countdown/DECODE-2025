package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;

@Autonomous(name="Shoot 3 and Intake 3", group="Robot")
@Config
public class Auto3 extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;

    //102.22
    //91.44
    //30.48
    public static double BEFORE_TAPE = 84;
    public static double AFTER_TAPE = 132;
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
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            // Need to add the shooting of preload
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, BEFORE_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, AFTER_TAPE, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(15000);
        } else {
            // Need to add the shooting of preload
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -BEFORE_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, TAPE_LOW, -AFTER_TAPE, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(15000);
        }

        // Need to add the shooting of preload
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0.3), 1);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP /= 2, 1);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setPower(0), 2);
        robotContainer.delayedActionManager.schedulePose(() -> Constants.Pathing.LATITUDE_KP *= 2, 2);

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.spindexer.shootNextBall(), 3);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootNextBall(), 4000), 3);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.shootNextBall(), 8000), 3);

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


        double spindexerError = Math.abs(robotContainer.spindexer.pdf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - spindexerError) > 50) {
            spindexAccel.reset();
        }

        if (spindexerError > 2) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pdf.calculate() * spindexAccel.seconds(), 1));
            } else {
                robotContainer.spindexer.setPower(robotContainer.spindexer.pdf.calculate());
            }
        } else {
            robotContainer.spindexer.setPower(0);
        }
        lastError = spindexerError;

        if (robotContainer.beamBreakToggleButton.wasJustReleased()) {
            robotContainer.spindexer.goToNextIntakeSlot();
        }

        if (Status.turretToggleButton.wasJustPressed()) {
            robotContainer.spindexer.goToNextTransferSlot();
        }

       // This LimeLight speed setting for the flywheel speed might not be working for auto.
       if (!Status.intakeToggle) {
           robotContainer.turret.flywheel.setTargetVelocity(Math.min(Math.pow(Status.turretToggleButton.getHoldDuration(), Constants.Turret.FLYWHEEL_CURVE), robotContainer.limelightLogic.getRequiredFlywheelSpeed()));
       } else {
           robotContainer.turret.flywheel.setTargetVelocity(0);
       }
       blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
