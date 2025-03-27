package org.firstinspires.ftc.teamcode;

public class DrivetrainThread extends Thread {
    private Robot robot;

    public DrivetrainThread(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void run() {
        while (robot.running) {
            for (int i = 0; i < robot.swerveModules.length; i++) {
                SwerveModule module = robot.swerveModules[i];
                if (!robot.running) {
                    break;
                }
                robot.swerveServosPIDF[i].calculate();
            }

            try {
                Thread.sleep(10); // prevent CPU strain
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
