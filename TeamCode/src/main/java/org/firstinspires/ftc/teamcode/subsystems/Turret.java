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
import org.firstinspires.ftc.teamcode.util.PIDF;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final BetterServo hoodServo;
    private FlywheelPDF flywheelPDF;
    private PIDF flywheelPIDF;
    private double targetPosition;
    private double targetPositionDegrees;
    private double manualTurretPos;
    private double turretAngleOffset;
    private GamepadWrapper.ButtonReader backFieldButton = new GamepadWrapper.ButtonReader();

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, BetterServo hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
        this.flywheelPDF = new FlywheelPDF(robotContainer, flyWheelMotors);
        // this.flywheelPIDF = new PIDF(robotContainer, Constants.Turret.FLYWHEEL_KP, Constants.Turret.FLYWHEEL_KI, Constants.Turret.FLYWHEEL_KD, Constants.Turret.FLYWHEEL_KF);
        this.targetPosition = 0;
        this.manualTurretPos = 0;
        this.turretAngleOffset = 0;
    }

    public void update(boolean teleop) {
        // flywheelPIDF = flywheelPIDF.updateValues(robotContainer, Constants.Turret.FLYWHEEL_KP, Constants.Turret.FLYWHEEL_KI, Constants.Turret.FLYWHEEL_KD, Constants.Turret.FLYWHEEL_KF);

        Status.turretToggleButton.update(Status.turretToggle);

        if (teleop) {
            if (robotContainer.gamepadEx2.dpadLeft.wasJustReleased()) {
                turretAngleOffset -= 3;
            }

            if (robotContainer.gamepadEx2.dpadRight.wasJustReleased()) {
                turretAngleOffset += 3;
            }

            if (!Status.intakeToggle) {
                flywheel.targetVelocity = Math.min(Status.turretToggleButton.holdDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
            } else {
                flywheel.targetVelocity = 0;
            }

            // Manual turret hood
            if (robotContainer.gamepadEx1.circle.isPressed() || Status.currentPose.getX(DistanceUnit.CM) < -75) {
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

            // Automated flywheel
            if (robotContainer.gamepadEx2.circle.wasJustReleased() && !Status.intakeToggle) {
                Status.turretToggle = true;
            } else if (robotContainer.gamepadEx2.circle.wasJustReleased() && Status.intakeToggle) {
                Status.turretToggle = false;
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

            if (!Status.intakeToggle) {
                flywheel.targetVelocity = Math.min(Status.turretToggleButton.holdDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
            } else {
                flywheel.targetVelocity = 0;
            }
        }

        flywheel.targetVelocity = flywheel.targetVelocity * Constants.Turret.FLYWHEEL_MAX_VELOCITY;
        Status.flywheelAtTargetSpeed = robotContainer.turret.flywheel.atTargetVelocity();
        double targetPower = flywheelPDF.calculate(flywheel.targetVelocity);
        // double targetPower = flywheelPIDF.update((flywheel.targetVelocity));
        robotContainer.telemetry.addData("Flywheel Target Speed", flywheel.targetVelocity);
        robotContainer.telemetry.addData("Flywheel Power", targetPower);
        flyWheelMotors.setPower(targetPower);
    }

    // This is for manual control
    public void setTargetPosition(double position) {
        targetPosition = position;
        turretServos.setPosition(HelperFunctions.clamp(position, Constants.Turret.TURRET_MIN, Constants.Turret.TURRET_MAX));
    }

    public void setTargetAngle(double angleInDegrees) { // angleInDegrees should be between -180 and 180
        targetPositionDegrees = angleInDegrees;
        angleInDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffset);
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
        robotContainer.telemetry.addData("Goal x cm", Status.goalPose.getX(DistanceUnit.CM));
        robotContainer.telemetry.addData("Goal y cm", Status.goalPose.getY(DistanceUnit.CM));
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
