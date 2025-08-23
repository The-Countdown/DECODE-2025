package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.PathPlanner;
import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.Marker;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Constants;

@Autonomous(name="Pathplanner example", group="Robot")
public class PathPlannerExample extends LinearOpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    public static boolean fieldOriented = false;
    private int currentServo = -1;

    private PathPlanner pathplanner;


    @Override
    public void runOpMode() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU(); // TODO: Run at start of auto instead
        robotContainer.drivetrain.swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
        robotContainer.telemetry.addLine("Auto Initialized");
        robotContainer.telemetry.update();
        pathplanner = new PathPlanner(robotContainer.telemetry);


        // Do stuff
        pathplanner.addMarker(new Marker(new Pose2D(DistanceUnit.CM, 100, 100, AngleUnit.DEGREES, 300)));
        pathplanner.addMarker(new Marker(new Pose2D(DistanceUnit.CM, 200, 200, AngleUnit.DEGREES, 300)));

        waitForStart();

        while (opModeIsActive()) {
            pathplanner.run();
        }
    }
}
