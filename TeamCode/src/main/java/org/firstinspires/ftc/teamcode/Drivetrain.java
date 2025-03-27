package org.firstinspires.ftc.teamcode;

public class Drivetrain extends Robot.HardwareDevices {
    private Robot robot;



    public Drivetrain(Robot robot) {
        this.robot = robot;
    }

    public class Servo {
        //Gets the positions in degrees of the swerve module servos, 0 being forwards
        public double[] getSwerveServoAngles() {
            double[] positions = new double[4];
            for (int i = 0; i < positions.length; i++) {
                robot.refreshData();
                positions[i] = swerveAnalogs[i].getVoltage() / 3.3 * 360 + swerveServoAngleOffset[4] + swerveServoAngleOffset[i];
            }

            return positions;
        }

        //Sets the positions in degrees of the swerve module servos, 0 being forwards
        public void setServoPowers(double[] servoPositions) {
            for (int i = 0; i < servoPositions.length; i++) {
                swerveServos[i].setPower(servoPositions[i]);
            }
        }

        public void setTargetServoAngles(double[] targetAngles, double power) {
            double[] currentAngles = getSwerveServoAngles();

            boolean allReached = false;

            while (!allReached) {
                allReached = true;

                for (int i = 0; i < targetAngles.length; i++) {
                    double currentAngle = currentAngles[i];
                    double targetAngle = targetAngles[i];
                    boolean increasing = targetAngle > currentAngle;
                    double adjustedPower = increasing ? Math.abs(power) : -Math.abs(power);

                    if ((increasing && currentAngle < targetAngle) || (!increasing && currentAngle > targetAngle)) {
                        swerveServos[i].setPower(adjustedPower);
                        allReached = false;
                    } else {
                        swerveServos[i].setPower(0);
                    }
                }
                currentAngles = getSwerveServoAngles();
            }
        }
    }

    public class motor {
        public void setMotorPowers(double[] motorPowers) {
            for (int i = 0; i < motorPowers.length; i++) {
                swerveMotors[i].setPower(motorPowers[i]);
            }
        }
    }

    public Servo servo = new Servo();
}
