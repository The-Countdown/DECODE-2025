package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

import org.firstinspires.ftc.teamcode.hardware.BetterServo;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final BetterServo hoodServo;
    private double targetPosition = 0;
    private double[] turretPositionTable = {0.785, 0.50, 0.2225}; // -90, 0, 90

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, BetterServo hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
    }

    // TODO: Limit angle of rotation, and have that be configurable in constants

    public void setTargetPosition(double position) {
        targetPosition = position;
        turretServos.setPosition(HelperFunctions.clamp(((position + 1) / 2), Constants.Turret.TURRET_LIMIT_MIN_SERVO, Constants.Turret.TURRET_LIMIT_MAX_SERVO));
    }

    public void setTargetRaw(double position) {
        turretServos.setPosition(position);
    }
//    public void setTargetAngle(double angle) {
//        turretServos.setPosition((HelperFunctions.clamp(angle > 355 || angle < 0 ? (angle + 355) % 355 : angle, Constants.Turret.TURRET_LIMIT_MIN_ANGLE, Constants.Turret.TURRET_LIMIT_MAX_ANGLE)) / 355);
//    }
    public void setTargetAngle(double angleInDegrees) { // angleInDegrees should be between -180 and 180
        if (angleInDegrees > 90 || angleInDegrees < -90) {
            return;
        }
        if (angleInDegrees < 0) { // If angle between -90 and 0
            turretServos.setPosition(HelperFunctions.interpolate(turretPositionTable[0], turretPositionTable[1], ((angleInDegrees + 90) / 90)));
        } else { // If angle between 0 and 90
            turretServos.setPosition(HelperFunctions.interpolate(turretPositionTable[1], turretPositionTable[2], ((angleInDegrees - 90) / 90) + 1));
        }
    }

    public double getPosition() {
        return turretServos.getPosition();
    }


    public void pointAtGoal() {
        double xDiff;
        double yDiff;
        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            xDiff = Status.GOAL_POSE.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            yDiff = Status.GOAL_POSE.getY(DistanceUnit.CM) + Status.currentPose.getY(DistanceUnit.CM);
        } else {
            xDiff = Status.GOAL_POSE.getX(DistanceUnit.CM) + Status.currentPose.getX(DistanceUnit.CM);
            yDiff = Status.GOAL_POSE.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        }
        // double angleToFaceGoal = Math.toDegrees(Math.atan2(yDiff, xDiff)) - Status.currentHeading;
        // if (!Double.isNaN(angleToFaceGoal)) {
        //     setTargetAngle(angleToFaceGoal);
        //     robotContainer.telemetry.addData("Angle To Face Goal", angleToFaceGoal);
        // }
        //
        robotContainer.telemetry.addData("Goal x cm", Status.GOAL_POSE.getX(DistanceUnit.CM));
        robotContainer.telemetry.addData("Goal y cm", Status.GOAL_POSE.getY(DistanceUnit.CM));
        robotContainer.telemetry.addData("Robot x cm", Status.currentPose.getX(DistanceUnit.CM));
        robotContainer.telemetry.addData("Robot y cm", Status.currentPose.getY(DistanceUnit.CM));
        robotContainer.telemetry.addData("x Diff", xDiff);
        robotContainer.telemetry.addData("y Diff", yDiff);

        double angleToFaceGoal = ((Math.atan(yDiff / xDiff) * (180 / Math.PI)) + Status.currentHeading - 180);
        setTargetAngle(HelperFunctions.normalizeAngle(angleToFaceGoal));
        robotContainer.telemetry.addData("Angle To Face Goal", angleToFaceGoal);
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < 10;
    }

    public class Flywheel {
        double targetVelocity = 0;
        public void setTargetVelocity(double power) {
            targetVelocity = Constants.Turret.FLYWHEEL_MAX_VELOCITY * power;
            if (Status.flywheelToggle) {
                flyWheelMotors.setVelocity(targetVelocity);
            } else {
                flyWheelMotors.setVelocity(0);
            }
        }

        public double interpolateByDistance(double disToGoal){
            double lowerPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[0];
            int lowerPointIndex = 0;
            double higherPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1];
            int higherPointIndex = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1;
            double currentDistance;
            for (int i = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length; i>0;i--){
                currentDistance = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[i-1];
                if(currentDistance > disToGoal){
                    if (currentDistance < higherPoint) {
                        higherPoint = currentDistance;
                        higherPointIndex = i;
                    }
                }else if (currentDistance < disToGoal){
                    if (currentDistance > lowerPoint) {
                        lowerPoint = currentDistance;
                        lowerPointIndex = i;
                    }
                }else if (currentDistance == disToGoal){
                    return Constants.Turret.FLYWHEEL_SPEED_TABLE[i];
                }
            }
            lowerPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE[lowerPointIndex];
            higherPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE[higherPointIndex];
            return HelperFunctions.interpolate(lowerPoint, higherPoint, (disToGoal-lowerPoint)/(higherPoint-lowerPoint));
        }

        public boolean atTargetVelocity() {
            return flyWheelMotors.getAverageVelocity() - targetVelocity < 100;
        }

        public double getFlywheelVelocity() {
            return flyWheelMotors.getAverageVelocity();
        }
    }

    public class Hood {
        public void setPos(double pos) {
            hoodServo.updateSetPosition(pos);
        }
    }

    public final Flywheel flywheel = new Flywheel();
    public final Hood hood = new Hood();
}
