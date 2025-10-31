package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final ServoImplEx hoodServo;
    private double targetPosition = 0;

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, ServoImplEx hoodServo, LinkedServos turretServos) {
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
    public void setTargetAngle(double angle) {
        turretServos.setPosition((HelperFunctions.clamp(angle > 355 || angle < 0 ? (angle + 355) % 355 : angle, Constants.Turret.TURRET_LIMIT_MIN_ANGLE, Constants.Turret.TURRET_LIMIT_MAX_ANGLE)) / 355);
    }

    public double getPosition() {
        return turretServos.getPosition();
    }


    public void pointAtGoal() {
        double xDiff = Constants.Game.GOAL_POSE.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
        double yDiff = Constants.Game.GOAL_POSE.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        double angleToFaceGoal = Math.toDegrees(Math.atan2(yDiff, xDiff)) - Status.currentHeading;
        if (!Double.isNaN(angleToFaceGoal)) {
            setTargetAngle(angleToFaceGoal);
            robotContainer.telemetry.addData("Angle To Face Goal", angleToFaceGoal);
        }
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < 10;
    }

    public class Flywheel {
        double targetVelocity = 0;
        public void setTargetVelocity(double power) {
            targetVelocity = Constants.Turret.FLYWHEEL_MAX_VELOCITY * power;
            flyWheelMotors.setVelocity(targetVelocity);
        }

        public boolean atTargetVelocity() {
            return flyWheelMotors.getAverageVelocity() - targetVelocity < 100;
        }
    }

    public class Hood {
//        public void setTargetAngle(double angle) {
//            if (angle > 45) {
//                angle = 45;
//            } else if (angle < 10) {
//                angle = 10;
//            }
//            hoodServo.setPosition(angle/355);
//        }

        public void setPos(double pos) {
            hoodServo.setPosition(pos);
        }
    }

    public final Flywheel flywheel = new Flywheel();
    public final Hood hood = new Hood();
}
