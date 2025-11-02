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

@Autonomous(name="Auto", group="Robot")
public class Auto extends OpMode {
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private final ElapsedTime spindexAccel = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.init();
//        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        blackboard.put("pose", Status.currentPose);
        robotContainer.telemetry.addData("Alliance Color", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
        robotContainer.telemetry.update();

        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Constants.Robot.startingX + 70, Constants.Robot.startingY - 30, AngleUnit.DEGREES, 0));
        } else {
            robotContainer.pathPlanner.addPose(new Pose2D(DistanceUnit.CM, Constants.Robot.startingX + 70, Constants.Robot.startingY + 30, AngleUnit.DEGREES, 0));
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

        robotContainer.turret.pointAtGoal();
        robotContainer.turret.flywheel.setTargetVelocity(0.6);
        robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.goToNextTransferSlot(), 1000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapUp(), 2000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), 2200);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.goToNextTransferSlot(), 3000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapUp(), 4000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), 4200);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.spindexer.goToNextTransferSlot(), 5000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapUp(), 6000);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.transfer.flapDown(), 6200);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.turret.flywheel.setTargetVelocity(0), 6300);
        robotContainer.delayedActionManager.schedule(() -> robotContainer.pathPlanner.driveToPose(0), 10000);
        robotContainer.telemetry.addData("Pose 1", Status.currentPose);
    }

    @Override
    public void loop() {
        robotContainer.delayedActionManager.update();

        double spindexerError = Math.abs(robotContainer.spindexer.pidf.getError());
        // If the error changes by a lot in a short period of time reset the timer

        if (Math.abs(lastError - spindexerError) > 50) {
            spindexAccel.reset();
        }

        if (spindexerError > 2) {
            if (spindexAccel.seconds() <= 1) {
                robotContainer.spindexer.setPower(Math.min(robotContainer.spindexer.pidf.calculate() * spindexAccel.seconds(), 0.5));
            } else {
                robotContainer.spindexer.setPower(robotContainer.spindexer.pidf.calculate());
            }
        } else {
            robotContainer.spindexer.setPower(0);
        }
        lastError = spindexerError;
//        blackboard.put("pose", Status.currentPose);
    }

    @Override
    public void stop() {
        robotContainer.completedAuto = true;
        blackboard.put("pose", Status.currentPose);
                
        robotContainer.localizationUpdater.stopLocalizer();
        try {
            robotContainer.localizationUpdater.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        robotContainer.drivetrainUpdater.stopEnabled();
        try {
            robotContainer.drivetrainUpdater.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
