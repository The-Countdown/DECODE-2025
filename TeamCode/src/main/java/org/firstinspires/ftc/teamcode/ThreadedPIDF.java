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
    private final RobotManager robotManager;

    public ThreadedPIDF(RobotManager robotManager) {
        this.robotManager = robotManager;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("ThreadedPIDF");
    }

    @Override
    public void run() {
        robotManager.refreshData();
        boolean skip = false;
        while (robotManager.isRunning) {
            for (int i = 0; i < robotManager.swerveModules.length; i++) {
                if (!robotManager.isRunning) {
                    break;
                }

                if (robotManager.swerveServosPIDF[i].getError() <= Constants.SWERVE_SERVO_PIDF_TOLERANCE_DEGREES) {
                    Status.swerveServoStatus.put(i, Status.ServoStatus.TARGET_REACHED);
                    skip = true;
                    break;
                }
                Status.swerveServoStatus.put(i, Status.ServoStatus.MOVING);

                robotManager.swerveModules[i].servo.setPower(robotManager.swerveServosPIDF[i].calculate());
            }

            try {
                robotManager.refreshData();

                if (!skip) {
                    Thread.sleep(5); // prevent CPU strain
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
