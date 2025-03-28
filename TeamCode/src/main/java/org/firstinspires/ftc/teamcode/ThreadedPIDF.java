package org.firstinspires.ftc.teamcode;

public class ThreadedPIDF extends Thread {
    private Robot robot;

    public ThreadedPIDF(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void run() {
        robot.refreshData();
        boolean skip = false;
        while (robot.running) {
            for (int i = 0; i < robot.swerveModules.length; i++) {
                if (!robot.running) {
                    break;
                }

                if (robot.swerveServosPIDF[i].getError() <= 2) {
                    Status.SERVO_TARGET_REACHED[i] = true;
                    skip = true;
                    break;
                }
                Status.SERVO_TARGET_REACHED[i] = false;

                robot.swerveModules[i].servo.setPower(robot.swerveServosPIDF[i].calculate());
            }

            try {
                robot.refreshData();

                if (!skip) {
                    Thread.sleep(10); // prevent CPU strain
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
