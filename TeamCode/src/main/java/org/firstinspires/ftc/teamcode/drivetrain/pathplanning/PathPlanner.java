package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;

import java.util.ArrayList;
import java.lang.Thread;

public class PathPlanner {
    Telemetry telemetry;
    RobotContainer robot;

    Pose2D target;
    ArrayList<Marker> markers = new ArrayList<Marker>();

    public PathPlanner(Telemetry telemetry, RobotContainer robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void addMarker(Marker marker) {
        markers.add(marker);
    }

    public Telemetry getTelemetry() {
    }

    public void driveToPos(int nu, Marker[] markers) {
        Pose2D currentPose = PinpointUpdater.currentPose;
        Pose2D targetPose = markers[nu].getPose();

        double deltaX = targetPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double deltaY = targetPose.getY(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);

        double angleToTarget = Math.atan2(deltaY, deltaX);
        double[] angles = {angleToTarget, angleToTarget, angleToTarget, angleToTarget};

        double[] powers = {0.5,0.5,0.5,0.5};

        robot.drivetrain.swerveSetTargets(angles,powers);
        //array list -> markers -> pose2D -> variables i need;
    }

    public void run() {
        for (int i = 0; i < markers.size(); i++) {
            telemetry.addData("Marker", markers.get(i).markerPose.toString());
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (Exception error) {
            }
        }
    }

    //drive to pos
    // make input int bc it need to take number from array to go to

}
