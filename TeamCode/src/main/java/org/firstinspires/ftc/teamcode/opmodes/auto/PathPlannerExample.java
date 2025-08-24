package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

@Autonomous(name="Pathplanner example", group="Robot")
public class PathPlannerExample extends LinearOpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;

    @Override
    public void runOpMode() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.telemetry.addLine("Auto Initialized");
        robotContainer.telemetry.update();
        robotContainer.pathPlanner = new PathPlanner(robotContainer.telemetry, robotContainer);

        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 100, 100, AngleUnit.DEGREES, 300));
        robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 200, 200, AngleUnit.DEGREES, 300));

        waitForStart();
        
        //ELLIUJOTTTT how would i make it not a huge while loop for going to one target pose
        if (opModeIsActive()) {
            robotContainer.pathPlanner.driveToPose(0);
            robotContainer.pathPlanner.driveToPose(1);
        }
    }
}
