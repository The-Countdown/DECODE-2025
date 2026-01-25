package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.FlywheelPDF;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final BetterServo hoodServo;
    private FlywheelPDF flywheelPDF;
    private double targetPosition;
    private double targetPositionDegrees;
    private double manualTurretPos;
    private double turretAngleOffset;
    private double turretAngleOffsetFar;
    private GamepadWrapper.ButtonReader backFieldButton = new GamepadWrapper.ButtonReader();

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, BetterServo hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
        this.flywheelPDF = new FlywheelPDF(robotContainer, flyWheelMotors);
        this.targetPosition = 0;
        this.manualTurretPos = 0;
        this.turretAngleOffset = 0;
        this.turretAngleOffsetFar = 0;
    }

    public void update(boolean teleop) {
        Status.flywheelToggleButton.update(Status.flywheelToggle);

        if (Status.flywheelToggle) {
            flywheel.targetVelocity = Math.min(Status.flywheelToggleButton.holdDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
        } else {
            flywheel.targetVelocity = 0;
        }

        flywheel.targetVelocity = flywheel.targetVelocity * Constants.Turret.FLYWHEEL_MAX_VELOCITY;
        Status.flywheelAtTargetSpeed = robotContainer.turret.flywheel.atTargetVelocity();
        double targetPower = flywheelPDF.calculate(flywheel.targetVelocity);
        // Only run flywheel if good voltage
        if (robotContainer.controlHubVoltage > 8) {
            flyWheelMotors.setPower(targetPower);
        }
        if (teleop) {
            // Change this to change the Status.change degree whatever to rotate the robot pose, but this will need to be changed in the robot
            // So some though will be required
            if (robotContainer.gamepadEx2.dpadLeft.wasJustReleased()) {
                if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
                    turretAngleOffsetFar -= 3;
                }
                turretAngleOffset -= 3;
            }

            if (robotContainer.gamepadEx2.dpadRight.wasJustReleased()) {
                if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
                    turretAngleOffsetFar += 3;
                }
                turretAngleOffset += 3;
            }

            // Manual turret hood
            if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) < 20) && Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) > -20) && Status.alliance == Constants.Game.ALLIANCE.RED) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
            } else {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
            }

            backFieldButton.update(Status.currentPose.getX(DistanceUnit.CM) < -20);

            if (backFieldButton.wasJustPressed()) {
                if (Status.alliance == Constants.Game.ALLIANCE.RED) {
                    Status.goalPose = new Pose2D(DistanceUnit.INCH, 87, 70, AngleUnit.DEGREES, -45);
                } else {
                    Status.goalPose = new Pose2D(DistanceUnit.INCH, -87, 70, AngleUnit.DEGREES, 45);
                }
            } else if (backFieldButton.wasJustReleased()) {
                if (Status.alliance == Constants.Game.ALLIANCE.RED) {
                    Status.goalPose = new Pose2D(DistanceUnit.INCH, 70, 68, AngleUnit.DEGREES, -45);
                } else {
                    Status.goalPose = new Pose2D(DistanceUnit.INCH, -70, 68, AngleUnit.DEGREES, 45);
                }
            }

            // Turret turn - Right stick X
             if (Status.manualControl && robotContainer.gamepadEx2.rightStickX() != 0) {
                 // Manual turret turning
                 manualTurretPos -= robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.TURRET_SPEED_FACTOR * robotContainer.CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
                 manualTurretPos = HelperFunctions.clamp(manualTurretPos, Constants.Turret.TURRET_MIN, Constants.Turret.TURRET_MAX);
                 robotContainer.turret.setTargetPosition(manualTurretPos);
             } else if (Status.manualControl) {
             } else {
                 robotContainer.turret.pointAtGoal();
             }

        } else {
            if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 15 || Status.currentPose.getY(DistanceUnit.CM) < 15) && Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 15 || Status.currentPose.getY(DistanceUnit.CM) > -15) && Status.alliance == Constants.Game.ALLIANCE.RED) {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
            } else {
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
            }
        }
    }

    // This is for manual control
    public void setTargetPosition(double position) {
        targetPosition = position;
        turretServos.setPosition(HelperFunctions.clamp(position, Constants.Turret.TURRET_MIN, Constants.Turret.TURRET_MAX));
    }

    public void setTargetAngle(double angleInDegrees) { // angleInDegrees should be between -180 and 180
        targetPositionDegrees = angleInDegrees;
        if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
            angleInDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffsetFar);
        } else {
            angleInDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffset);
        }
        if (angleInDegrees < Constants.Turret.TURRET_LIMIT_MIN_ANGLE || angleInDegrees > Constants.Turret.TURRET_LIMIT_MAX_ANGLE) {
            return;
        }

        // turretServos.setPositionDegrees(angleInDegrees);

        // Legacy interpolate
        if (angleInDegrees < 0) { // If angle between -90 and 0
            turretServos.setPosition(HelperFunctions.interpolate(Constants.Turret.TURRET_MAX, Constants.Turret.TURRET_NEUTRAL, ((angleInDegrees + 90) / 90)));
        } else { // If angle between 0 and 90
            turretServos.setPosition(HelperFunctions.interpolate(Constants.Turret.TURRET_NEUTRAL, Constants.Turret.TURRET_MIN, ((angleInDegrees - 90) / 90) + 1));
        }
    }

    public double getPosition() {
        return turretServos.getPosition();
    }

    public double getPositionDegrees() {
        return targetPositionDegrees;
    }

    public void pointAtGoal() {
        double xDiff;
        double yDiff;
        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            xDiff = Status.goalPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            yDiff = Status.goalPose.getY(DistanceUnit.CM) + Status.currentPose.getY(DistanceUnit.CM);
        } else {
            xDiff = Status.goalPose.getX(DistanceUnit.CM) + Status.currentPose.getX(DistanceUnit.CM);
            yDiff = Status.goalPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        }
        double angleToFaceGoal = ((Math.atan(yDiff / xDiff) * (180 / Math.PI)) + Status.currentHeading - 180);
        setTargetAngle(HelperFunctions.normalizeAngle(angleToFaceGoal));
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < 10;
    }

    public class Flywheel {
        public double targetVelocity = 0;
        public double targetMaxVelocity = 0;

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

        public void updateLaunchValues(double distToGoal){
            double verticalVel = Math.sqrt(2 * Constants.Game.GRAVITY * (Constants.Turret.DESIRED_MAX_HEIGHT - Constants.Turret.FLYWHEEL_HEIGHT));
            double estimatedTime = (verticalVel + Math.sqrt(verticalVel - 2 * (Constants.Game.GRAVITY) * (Constants.Game.GOAL_HEIGHT - Constants.Turret.FLYWHEEL_HEIGHT))) / Constants.Game.GRAVITY;
            double horizonalVel = distToGoal / estimatedTime;
            double hoodAngle = Math.atan(verticalVel / horizonalVel);
            double xDiff;
            double yDiff;
            if (Status.alliance == Constants.Game.ALLIANCE.BLUE){
                 xDiff = -Status.goalPose.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
                 yDiff = Status.goalPose.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            } else {
                 xDiff = Status.goalPose.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
                 yDiff = -Status.goalPose.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            }
            double angleToFaceGoal = Math.atan2(yDiff, xDiff);
            double initialVel = Math.sqrt(Math.pow((verticalVel), 2) * Math.pow((horizonalVel), 2));
            double rpm = initialVel * (2000 / 5.14);
            targetVelocity = (rpm / 60) * Constants.Swerve.MOTOR_TICKS_PER_REVOLUTION;
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
