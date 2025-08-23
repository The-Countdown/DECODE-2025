package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.teamcode.drivetrain.pathplanning.Marker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.lang.Thread;

public class PathPlanner {
    Telemetry telemetry;
    ArrayList<Marker> markers = new ArrayList<Marker>();

    public PathPlanner(Telemetry telemetry) {
        telemetry = telemetry;
    }

    public void addMarker(Marker marker) {
        markers.add(marker);
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
}
