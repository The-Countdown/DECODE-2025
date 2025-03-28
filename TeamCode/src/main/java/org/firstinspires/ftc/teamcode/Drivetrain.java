package org.firstinspires.ftc.teamcode;

public class Drivetrain extends Robot.HardwareDevices {
    private Robot robot;

    public Drivetrain(Robot robot) {
        this.robot = robot;
    }

    public void driverControl(double x, double y, double rX) {

    }

    public void drvingInput(double[] targetAngles, double[] targetPowers) {
        for(int i = 0; i < swerveServos.length; i++) {
            if (targetAngles[i] < -180 || targetAngles[i] > 180) {
                throw new IllegalArgumentException("Input angle must be between -180 and 180, Input Angle: " + targetAngles[i] + "Module: " + i);
            }

            double currentAngle = robot.swerveModules[i].servo.getAngle();
            double error = targetAngles[i] - currentAngle;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            if (Math.abs(error) > 90) {
                targetAngles[i] = ((targetAngles[i] + 180) % 360 + 360) % 360 - 180;
                robot.swerveModules[i].motor.setPower(-targetPowers[i]);
            } else {
                robot.swerveModules[i].motor.setPower(targetPowers[i]);
            }

            robot.swerveModules[i].servo.setTargetAngle(targetAngles[i]);
        }
    }
}
