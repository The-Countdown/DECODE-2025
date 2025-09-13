package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.LinkedMotors;

public class Turret extends RobotContainer.HardwareDevices {
    RobotContainer robotContainer;
    LinkedMotors flyWheelMotors;
    ServoImplEx turretServo;
    ServoImplEx hoodServo;

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, ServoImplEx turretServo, ServoImplEx hoodServo) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.turretServo = turretServo;
        this.hoodServo = hoodServo;
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
    public void setTurretTargetAngle(double angle) {
        if (angle > 180) {
            angle = 180;
        } else if (angle < -180) {
            angle = -180;
        }
        turretServo.setPosition((angle + 180)/360);
    }

    //ticks per second
    //param 0-1
    //drive train updater go look at
    public void setFlywheelTargetVelocity(double power) {
        flyWheelMotors.setVelocity(Constants.SWERVE_MOTOR_MAX_VELOCITY_TICKS_PER_SECOND * power);
    }

    public void pointAtGoal() {
        //get the desired alliance color
        //get the target location of the goal
        //get the location of the robot
        //calculate the angle from the robot position to the goal using atan to get an angle of 0-90
        //take into account the rotation of the robot
        //finally aim the turret to the final angle or as close as possible
    }
}
