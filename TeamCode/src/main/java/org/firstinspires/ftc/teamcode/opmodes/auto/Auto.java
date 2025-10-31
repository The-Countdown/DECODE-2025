package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous(name="Auto", group="Robot")
public class Auto extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        blackboard.put("pose", Status.currentPose);
        blackboard.put("heading", Status.currentPose);

        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 0, 100, AngleUnit.DEGREES, 300));
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = true;

        robotContainer.start(this);

        robotContainer.turret.pointAtGoal();
        robotContainer.turret.flywheel.setTargetVelocity(robotContainer.limelightLogic.useInterpolate());
        robotContainer.pathPlanner.driveToPose(0);
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        robotContainer.completedAuto = true;
        blackboard.put("pose", Status.currentPose);
        blackboard.put("heading", Status.currentHeading);
    }
}
