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

@Autonomous(name="GoBackToStart", group="Robot")
public class GoBackToStart extends OpMode {
    private RobotContainer robotContainer;

    Pose2D
            RED_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, -12, AngleUnit.DEGREES, -45),
            BLUE_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 45);
    double BEFORE_TAPE = 89.22;
    double AFTER_TAPE = 129.22;
    double TAPE_LOW = 91.44;
    double TAPE_MID = -30.48;
    double TAPE_HIGH = 30.48;


    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        robotContainer.start(this, false);
        robotContainer.localizationUpdater = new LocalizationUpdater(robotContainer);
        robotContainer.localizationUpdater.start();
        RobotContainer.HardwareDevices.pinpoint.setPosition((Pose2D) blackboard.getOrDefault("pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)));

        robotContainer.pathPlanner.addPose(Status.startingPose);
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.updatePathStatus();
        robotContainer.pathPlanner.driveThroughPath();
    }

    @Override
    public void stop() {
        Status.wentBackToStart = true;
        blackboard.put("pose", Status.currentPose);
        robotContainer.stop();
    }
}
