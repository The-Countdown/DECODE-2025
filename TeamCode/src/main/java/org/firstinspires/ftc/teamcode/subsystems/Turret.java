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
    private RobotContainer robotContainer;
    private LinkedMotors flyWheelMotors;
    private LinkedServos turretServos;
    private ServoImplEx hoodServo;

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, ServoImplEx hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
    }

    //turret hood change by degree
    //turret turn with 180 - -180 degrees
    //flywheel turn input 0-1 output velocity

    //example angles 10-45
    //not taking into account the gear ratios
    public void setHoodTargetAngle(double angle) {
        if (angle > 45) {
            angle = 45;
        } else if (angle < 10) {
            angle = 10;
        }
        hoodServo.setPosition(angle/355);
    }

    //assuming 1:1 ratio for turret
    //need calculate gear ratio when go home

    public void setTurretTargetPosition(double position) {
        turretServos.setPosition((position + 1)/2);
    }

    public void setTurretTargetAngle(double angle) {
        turretServos.setPositionDegree(HelperFunctions.normalizeAngle(angle));
    }

    //ticks per second
    //param 0-1
    //drive train updater go look at
    public void setFlywheelTargetVelocity(double power) {
        flyWheelMotors.setVelocity(Constants.SWERVE_MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power);
    }

    public void setTurretServosPower(double power) {
        if (power > 1) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }
        turretServos.setPower(power);
    }

    public void pointAtGoal() {
        double xDiff = Constants.GOAL_POSE.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
        double yDiff = Constants.GOAL_POSE.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
        double angleToFaceGoal = Math.atan(yDiff/xDiff) - Status.currentHeading;
        setTurretTargetAngle(angleToFaceGoal);
    }
}
