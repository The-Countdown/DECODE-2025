package org.firstinspires.ftc.teamcode;

/**
 * The `ThreadedPIDF` class is responsible for managing the Proportional-Integral-Derivative-Feedforward (PIDF)
 * control loop for the robot's swerve drive servos in a separate thread. By running the PIDF calculations
 * in a dedicated thread, it ensures that the servo positions are updated regularly without blocking the main
 * thread, which can handle other tasks like sensor readings and user input. The class optimizes the
 * control loop by only calculating and updating the servo power if the error is outside of the set threshold,
 * improving efficiency.
 */
public class ThreadedPIDF extends Thread {
    private Robot robot;

    public ThreadedPIDF(Robot robot) {
        this.robot = robot;
        setDaemon(true);
        setName("ThreadedPIDF");
    }

    @Override
    public void run() {
        robot.refreshData();
        boolean skip = false;
        while (robot.isRunning) {
            for (int i = 0; i < robot.swerveModules.length; i++) {
                if (!robot.isRunning) {
                    break;
                }

                if (robot.swerveServosPIDF[i].getError() <= Constants.PIDF_TOLERANCE_DEGREES) {
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
