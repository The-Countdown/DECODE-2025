package org.firstinspires.ftc.teamcode;

import java.util.logging.Handler;

public class Drivetrain extends Robot.HardwareDevices {
    private Robot robot;

    public Drivetrain(Robot robot) {
        this.robot = robot;
    }

    public class Servo {
        public void setServoPositions(double[] angles) {
            for(int i = 0; i < swerveServos.length; i++) {
                robot.swerveModules[i].servo.setAngle(angles[i]);
            }
        }
    }

    public class Motor {
    }

    public Servo servo = new Servo();
    public Motor motor = new Motor();
}
