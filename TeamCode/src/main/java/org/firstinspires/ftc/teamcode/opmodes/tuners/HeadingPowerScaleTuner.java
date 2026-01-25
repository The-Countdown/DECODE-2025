package org.firstinspires.ftc.teamcode.opmodes.tuners;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "HeadingPowerScaleTuner", group = "Auto")
public class HeadingPowerScaleTuner extends OpMode {
    private RobotContainer robotContainer;
    private final ElapsedTime pathTimer = new ElapsedTime();
    ArrayList<String> distance = new ArrayList<>();
    ArrayList<String> time = new ArrayList<>();
    private final Pose2D start = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    private final Pose2D target = new Pose2D(DistanceUnit.CM, 200, 0, AngleUnit.DEGREES, 180);

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
        Status.opModeIsActive = true;
        Status.isDrivingActive = false;
        robotContainer.pathPlanner.addPose(start);
        robotContainer.pathPlanner.addPose(target);
        RobotContainer.HardwareDevices.pinpoint.recalibrateIMU();
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RobotContainer.HardwareDevices.pinpoint.setPosition(new Pose2D (DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        robotContainer.start(this, false);
        pathTimer.reset();
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.delayedActionManager.update();
        robotContainer.pathPlanner.driveThroughPath(pathTimer);
        robotContainer.positionProvider.update(false);

        distance.add(String.valueOf(Math.sqrt(Math.pow(Status.currentPose.getX(DistanceUnit.CM), 2) + Math.pow(Status.currentPose.getY(DistanceUnit.CM), 2))));
        time.add(String.valueOf(pathTimer.seconds()));
    }

    @Override
    public void stop() {
        robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
        StringBuilder csv = new StringBuilder();
        csv.append("distance, time\n"); // header

        for (int i = 0; i < distance.toArray(new String[0]).length; i++) {
            csv.append(distance.toArray(new String[0])[i])
                    .append(',')
                    .append(time.toArray(new String[0])[i])
                    .append('\n');
        }

         robotContainer.writeToFile("Acceleration_Heading Data Log.txt", csv.toString());

        Status.isDrivingActive = false;
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}
