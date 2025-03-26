package org.firstinspires.ftc.teamcode;

public class SwerveBase extends Robot.HardwareDevices {
    private Robot robot;

    //5 is global, and the rest are in order
    public double[] swerveServoAngleOffset = {0, 0, 0, 0, 0};

    public SwerveBase(Robot robot) {
        this.robot = robot;
    }

    //Gets the positions in degrees of the swerve module servos
    public double[] getSwerveServoAngles() {
        double[] positions = new double[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = swerveAnalogs[i].getVoltage() / 3.3 * 360 + swerveServoAngleOffset[5] + swerveServoAngleOffset[i + 1];
        }

        return positions;
    }

    public void setServoAngles(double[] servoPositions) {
        for (int i = 0; i < servoPositions.length; i++) {
            swerveServos[i].setPower(servoPositions[i]);
        }
    }

    public void setMotorPowers(double[] motorPowers) {
        for (int i = 0; i < motorPowers.length; i++) {
            swerveMotors[i].setPower(motorPowers[i]);
        }
    }
}
