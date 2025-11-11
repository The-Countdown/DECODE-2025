package org.firstinspires.ftc.teamcode.opmodes.auto;

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

@Autonomous(name="Auto2", group="Robot")
public class Auto2 extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;

    Pose2D
            RED_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, -12, AngleUnit.DEGREES, 45),
            BLUE_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, -45);

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        blackboard.put("pose", Status.currentPose);
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 91.44, 99.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 91.44, 109.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 30.48, 99.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 30.48, 109.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(BLUE_MIDDLE);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   30.48, 99.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   30.48, 109.22, AngleUnit.DEGREES, 90));
            robotContainer.pathPlanner.addPose(BLUE_MIDDLE);
        } else {//109.22
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 91.44, -99.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 91.44, -109.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(Status.startingPose);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 30.48, -99.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   - 30.48, -109.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(RED_MIDDLE);
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   30.48, -99.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM,   30.48, -109.22, AngleUnit.DEGREES, -90));
            robotContainer.pathPlanner.addPose(RED_MIDDLE);
        }

        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(1), 0);
        robotContainer.delayedActionManager.schedulePose(() -> robotContainer.intake.setVelocity(0), 1);
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        robotContainer.start(this, true);
        robotContainer.localizationUpdater = new LocalizationUpdater(robotContainer);
        robotContainer.localizationUpdater.start();

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        } else {
            RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
        }
        
//        robotContainer.pathPlanner.setTarget(1);
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setVelocity(0.8), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(2));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(2), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(2));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setVelocity(0), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.flywheel.setTargetVelocity(0.5), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(3), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(3));
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]), () -> robotContainer.pathPlanner.hasPreviousPathCompleted(4));
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus();
        robotContainer.turret.pointAtGoal();
        robotContainer.pathPlanner.driveThroughPath();

        double spindexerError = Math.abs(robotContainer.spindexer.pdf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - spindexerError) > 50) {
            spindexAccel.reset();
        }

        if (spindexerError > 2) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pdf.calculate() * spindexAccel.seconds(), 0.5));
            } else {
                robotContainer.spindexer.setPower(robotContainer.spindexer.pdf.calculate());
            }
        } else {
            robotContainer.spindexer.setPower(0);
        }
        lastError = spindexerError;
//        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
