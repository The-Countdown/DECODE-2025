package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.ArrayList;
import java.lang.Thread;

public class PathPlanner {
    private Telemetry telemetry;
    private RobotContainer robotContainer;
    private DelayedActionManager delayedActionManager;
    ArrayList<Pose2D> poses = new ArrayList<>();
    boolean atTarget;
    double tolerance = 1;
    private static final ElapsedTime stopTimer = new ElapsedTime();
    double[] calculatedAngles = new double[Constants.Swerve.NUM_SERVOS];
    double[] calculatedPowers = new double[Constants.Swerve.NUM_MOTORS];
    double[] lastAngles = Constants.Swerve.STOP_FORMATION;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(pose);
    }

    /**
    * Calculates angle of the target relative to the current position of the robotContainer and drives to target
    * @param index which pose to drive to from first to last
    */
    public void driveToPose(int index) {
        atTarget = false;
        while (!atTarget) {
            Status.targetPose = poses.get(index);
            robotContainer.telemetry.addData("Current Position X: ", Status.currentPose.getX(DistanceUnit.CM));
            robotContainer.telemetry.addData("Current Position Y: ", Status.currentPose.getY(DistanceUnit.CM));

            robotContainer.telemetry.addData("diff X", Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM));
            robotContainer.telemetry.addData("diff Y", Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM));
            robotContainer.telemetry.update();

            // Know when the robotContainer is there
            if (Status.currentPose.getX(DistanceUnit.CM) + tolerance >= Status.targetPose.getX(DistanceUnit.CM) && Status.currentPose.getX(DistanceUnit.CM) - tolerance <= Status.targetPose.getX(DistanceUnit.CM)) { // Check X
                if (Status.currentPose.getY(DistanceUnit.CM) + tolerance >= Status.targetPose.getY(DistanceUnit.CM) && Status.currentPose.getY(DistanceUnit.CM) - tolerance <= Status.targetPose.getY(DistanceUnit.CM)) { // Check Y
                    atTarget = true;
                }
            }

            double deltaX = Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            double deltaY = Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);

//            double angleToTarget = -Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90;
            double angleToTarget = -Math.toDegrees(Math.atan2(deltaY, deltaX));
            double[] angles = {angleToTarget, angleToTarget, angleToTarget, angleToTarget};

            double[] powers;
            // Slow down if close to the target.
            if (deltaX > 5 || deltaY > 5) {
                powers = new double[]{0.1, 0.1, 0.1, 0.1};
            } else {
                powers = new double[]{0.1, 0.1, 0.1, 0.1};
            }

            robotContainer.drivetrain.setTargets(angles, powers);
//            robotContainer.pathPlanner.waitForTarget();
        }
        double[] emptyAngles = {0, 0, 0, 0};
        robotContainer.drivetrain.setTargets(emptyAngles, Constants.Swerve.NO_POWER);
    }

    public void setDirectionPowers(double x, double y, double rX) {
        double rotationalMagnitude = Math.abs(rX);

        if (Status.robotHeadingTargetReached && x == 0 && y == 0 && rX == 0 && stopTimer.seconds() >= 1) {
            robotContainer.drivetrain.setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
            return;
        }

        if (Status.robotHeadingTargetReached && x == 0 && y == 0 && rX == 0) {
            robotContainer.drivetrain.setTargets(lastAngles, Constants.Swerve.NO_POWER);
            return;
        }

        // Determine the rotational direction based on the sign of rX.
        int rotationalDirection = rX >= 0 ? 1 : -1;

        // Calculate the magnitude of translational movement.
        double translationalMagnitude = Math.sqrt(x * x + y * y);
        // Calculate the angle of translational movement.
        double translationalAngle = Math.atan2(y, x);
        // Set the initial translational direction to forward.
        int translationalDirection = 1;

        double currentHeading = HelperFunctions.normalizeAngle(Status.currentHeading + (Status.alliance == Constants.Game.ALLIANCE.RED ? 90 : -90));
        // Adjust the translational angle for field-oriented driving if enabled.
        translationalAngle = Status.fieldOriented ? translationalAngle + Math.toRadians(currentHeading) : translationalAngle;

        // Calculate the x and y components of translational movement.
        double translationalX = translationalMagnitude * Math.cos(translationalAngle) * translationalDirection;
        double translationalY = translationalMagnitude * Math.sin(translationalAngle) * translationalDirection;

        // Iterate through each swerve module to calculate its target angle and power.
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            // Calculate the x and y components of rotational movement.
            double rotationalX = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_COSINES_RADIANS[i] * rotationalDirection;
            double rotationalY = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_SINES_RADIANS[i] * rotationalDirection;

            // Combine the translational and rotational components into a single vector.
            double vectorX = translationalX + rotationalX;
            double vectorY = translationalY + rotationalY;

            // Calculate the magnitude and angle of the combined vector.
            double vectorMagnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);

            calculatedAngles[i] = Math.toDegrees(vectorAngle);
            calculatedPowers[i] = vectorMagnitude;
        }

        boolean anyNonZero = false;
        for (double v : calculatedAngles) {
            if (Math.abs(v) > 0.01) {
                anyNonZero = true;
                break;
            }
        }
        if (anyNonZero) {
            lastAngles = calculatedAngles;
        }

        robotContainer.drivetrain.setTargets(calculatedAngles, calculatedPowers);
    }

    public void driveThroughPath () {
//        delayedActionManager.run();
        for (int i = 0; i < poses.size(); i++) {
            driveToPose(i);
        }
    }

    public void waitForTarget() {
        while (!PoseMath.isAtPos()){
            Thread.yield();
        }
    }

    public void displayPoses() {
        for (int i = 0; i < poses.size(); i++) {
            telemetry.addData("Pose", poses.get(i).toString());
            telemetry.update();
        }
    }
}
