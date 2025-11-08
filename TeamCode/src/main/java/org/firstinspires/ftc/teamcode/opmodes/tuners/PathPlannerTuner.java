package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.LongitudePID;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PoseMath;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "PathPlannerTuner", group = "Tuner")
public class PathPlannerTuner extends OpMode {
    private RobotContainer robotContainer;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.setPosition((Pose2D) blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));
        RobotContainer.HardwareDevices.pinpoint.recalibrateIMU();
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, 12,12,AngleUnit.DEGREES, 90));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, -12,12,AngleUnit.DEGREES, 90));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, -12,-12,AngleUnit.DEGREES, 180));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.INCH, 12,-12,AngleUnit.DEGREES, 180));
    }

    @Override
    public void init_loop() {
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        Status.intakeToggle = false;
        Status.turretToggle = false;
        robotContainer.start(this);

        robotContainer.pathPlanner.setTarget(0);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(1), () -> robotContainer.pathPlanner.shouldSetTarget(1));
        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(2), () -> robotContainer.pathPlanner.shouldSetTarget(2));
        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(3), () -> robotContainer.pathPlanner.shouldSetTarget(3));


    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();

        if (robotContainer.pathPlanner.shouldSetTarget(4)) {
            robotContainer.pathPlanner.setTarget(0);
            robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(1), () -> robotContainer.pathPlanner.shouldSetTarget(1));
            robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(2), () -> robotContainer.pathPlanner.shouldSetTarget(2));
            robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.setTarget(3), () -> robotContainer.pathPlanner.shouldSetTarget(3));
        }

        telemetry.addData("y", Status.currentPose.getY(DistanceUnit.CM));
        telemetry.addData("x", Status.currentPose.getX(DistanceUnit.CM));
        telemetry.addData("angle", Status.currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Target: ", Status.targetPose.toString());
        telemetry.addData("Error: ", robotContainer.longitudePID.getError());
        Thread.yield();
    }

    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
