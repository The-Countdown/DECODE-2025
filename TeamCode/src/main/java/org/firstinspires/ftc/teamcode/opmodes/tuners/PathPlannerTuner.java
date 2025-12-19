package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PathPlannerTuner", group = "TeleOp")
public class PathPlannerTuner extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        Status.opModeIsActive = true;
        Status.startingPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        robotContainer.pathPlanner.addPose(Status.startingPose);
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 100, 0, AngleUnit.DEGREES, 90));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 100, 100, AngleUnit.DEGREES, 180));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 0, 100, AngleUnit.DEGREES, -90));

        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.startingPose);
    }

    @Override
    public void init_loop() {
        robotContainer.allIndicatorLights.rainbow();
    }

    @Override
    public void start() {
        robotContainer.start(this, true);
        Status.isDrivingActive = false;
        timer.reset();
    }

    @Override
    public void loop() {
        robotContainer.CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        robotContainer.DELTA_TIME_MS = robotContainer.CURRENT_LOOP_TIME_MS - robotContainer.PREV_LOOP_TIME_MS;
        robotContainer.refreshData();

        if (timer.seconds() < 1000) {
            robotContainer.pathPlanner.driveUsingPID(1);
        } else if (timer.seconds() < 2000) {
            robotContainer.pathPlanner.driveUsingPID(2);
        } else if (timer.seconds() < 3000) {
            robotContainer.pathPlanner.driveUsingPID(3);
        } else if (timer.seconds() < 4000) {
            robotContainer.pathPlanner.driveUsingPID(0);
        } else {
            timer.reset();
        }

        robotContainer.telemetry.update();
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);

        Status.isDrivingActive = false;
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
