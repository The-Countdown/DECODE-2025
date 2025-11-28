package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.FlywheelPDF;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

import org.firstinspires.ftc.teamcode.hardware.BetterServo;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final BetterServo hoodServo;
    private final FlywheelPDF flywheelPDF;
    private double targetPosition = 0;
    private double manualTurretPos = 0;
    public double[] turretPositionTable = {0.785, 0.50, 0.2225}; // -90, 0, 90

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, BetterServo hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
        this.flywheelPDF = new FlywheelPDF(robotContainer, flyWheelMotors);
    }

    public void update(boolean teleop, double CURRENT_LOOP_TIME_MS) {
        Status.turretToggleButton.update(Status.turretToggle);
        double flywheelTargetSpeed = 0;

        if (teleop) {
            if (!Status.intakeToggle) {
                flywheelTargetSpeed = Math.min(Status.turretToggleButton.getHoldDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
            } else {
                flywheelTargetSpeed = 0;
            }

            // Manual turret hood
            robotContainer.turret.hood.setPos(robotContainer.gamepadEx1.circle.isPressed() ? Constants.Turret.HOOD_PRESETS[1] : Constants.Turret.HOOD_PRESETS[0]);

            // Automated flywheel
            if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
                Status.turretToggle = true;
                robotContainer.spindexer.goToNextTransferSlot();
            } else if (robotContainer.gamepadEx2.circle.wasJustReleased() && Status.intakeToggle) {
                Status.turretToggle = false;
                robotContainer.spindexer.goToNextIntakeSlot(true);
            }

            // Turret turn - Right stick X
            if (Status.manualControl) {
                // Manual turret turning
                manualTurretPos -= robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
                manualTurretPos = HelperFunctions.clamp(manualTurretPos, Constants.Turret.TURRET_LIMIT_MIN, Constants.Turret.TURRET_LIMIT_MAX);
                robotContainer.turret.setTargetPosition(manualTurretPos);
            } else {
                // Automatic turret turning
                robotContainer.turret.pointAtGoal();
            }
        }

        Status.flywheelAtTargetSpeed = robotContainer.turret.flywheel.atTargetVelocity();
        double targetPower = flywheelPDF.calculate(flywheelTargetSpeed);
        flyWheelMotors.setPower(targetPower);
    }

    // This is for manual control
    public void setTargetPosition(double position) {
        targetPosition = position;
        turretServos.setPosition(HelperFunctions.clamp(((position + 1) / 2), Constants.Turret.TURRET_LIMIT_MIN_SERVO, Constants.Turret.TURRET_LIMIT_MAX_SERVO));
    }

    public void setTargetAngle(double angleInDegrees) { // angleInDegrees should be between -180 and 180
        if (angleInDegrees < Constants.Turret.TURRET_LIMIT_MIN_ANGLE || angleInDegrees > Constants.Turret.TURRET_LIMIT_MAX_ANGLE) {
            return;
        }

        turretServos.setPositionDegrees(angleInDegrees);

        // Legacy interpolate
        // if (angleInDegrees < 0) { // If angle between -90 and 0
        //     turretServos.setPosition(HelperFunctions.interpolate(turretPositionTable[0], turretPositionTable[1], ((angleInDegrees + 90) / 90)));
        // } else { // If angle between 0 and 90
        //     turretServos.setPosition(HelperFunctions.interpolate(turretPositionTable[1], turretPositionTable[2], ((angleInDegrees - 90) / 90) + 1));
        // }
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
        public double targetVelocity = 0;
        public double targetMaxVelocity = 0;

        // public void setTargetVelocity(double power) {
        //     targetVelocity = Constants.Swerve.MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power;
        //     if (Status.flywheelToggle) {
        //         flyWheelMotors.setVelocity(targetVelocity);
        //     } else {
        //         flyWheelMotors.setVelocity(0);
        //     }
        // }

        public void setPower(double power) {
            flyWheelMotors.setPower(targetVelocity);
        }

        public double interpolateByDistance(double disToGoal){
            double lowerPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[0];
            int lowerPointIndex = 0;
            double higherPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1];
            int higherPointIndex = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1;
            double currentDistance;
            for (int i = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-2; i>0;i--){
                currentDistance = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[i];
                if(currentDistance > disToGoal){
                    if (currentDistance < higherPoint) {
                        higherPoint = currentDistance;
                        higherPointIndex = i;
                    }
                }else if (currentDistance < disToGoal){
                    if (currentDistance >= lowerPoint) {
                        lowerPoint = currentDistance;
                        lowerPointIndex = i;
                    }
                }else if (currentDistance == disToGoal){
                    return Constants.Turret.FLYWHEEL_SPEED_TABLE[i];
                }
            }
            double lowerSpeed = Constants.Turret.FLYWHEEL_SPEED_TABLE[lowerPointIndex];
            double higherSpeed = Constants.Turret.FLYWHEEL_SPEED_TABLE[higherPointIndex];

            return HelperFunctions.interpolate(lowerSpeed, higherSpeed, (disToGoal-lowerPoint)/(higherPoint-lowerPoint));
        }

        public boolean atTargetVelocity() {
            return Math.abs(flyWheelMotors.getAverageVelocity() - targetVelocity) < 60 && targetVelocity > 10;
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
