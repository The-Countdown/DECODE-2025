package org.firstinspires.ftc.teamcode;

public class SwerveBase extends Robot.HardwareDevices {
    private Robot robot;

    //0 is global, and the rest are in order
    public double[] swerveServoAngleOffset = {0, 0, 0, 0, 0};

    public SwerveBase(Robot robot) {
        this.robot = robot;
    }

    //Gets the positions in degrees of the swerve module servos
    public double[] getSwerveServoAngles(){
        double[] positions = new double[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = swerveAnalogs[i].getVoltage() / 3.3 * 360 + swerveServoAngleOffset[0] + swerveServoAngleOffset[i + 1];
        }

        return positions;
    }
}
