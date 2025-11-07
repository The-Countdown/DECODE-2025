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
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        blackboard.put("pose", Status.currentPose);
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Constants.Robot.startingX + 70, Constants.Robot.startingY - 30, AngleUnit.DEGREES, 0));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 0,0, AngleUnit.DEGREES, 0));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, 0,0, AngleUnit.DEGREES, 0));

        } else {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Constants.Robot.startingX + 70, Constants.Robot.startingY + 30, AngleUnit.DEGREES, 0));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, -91.44, 109.22, AngleUnit.DEGREES, 89));
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, -30.48, 109.22, AngleUnit.DEGREES, 171));
        }
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        Status.lightsOn = true;
        Status.isDrivingActive = false;
        robotContainer.start(this);
        robotContainer.localizationUpdater = new LocalizationUpdater(robotContainer);
        robotContainer.localizationUpdater.start();
        RobotContainer.HardwareDevices.pinpoint.setPosition(Constants.Robot.startingPose);

        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.driveUsingPID(0),1000 );
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.driveUsingPID(1),2000 );
//        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.driveUsingPID(2),3000 );
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();
        robotContainer.turret.pointAtGoal();

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
    }
}
