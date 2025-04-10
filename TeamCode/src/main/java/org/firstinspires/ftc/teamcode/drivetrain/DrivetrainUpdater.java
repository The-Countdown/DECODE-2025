package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotManager;
import org.firstinspires.ftc.teamcode.main.Status;

/**
 * The `ThreadedPIDF` class is responsible for managing the Proportional-Integral-Derivative-Feedforward (PIDF)
 * control loop for the robot's swerve drive servos in a separate thread. By running the PIDF calculations
 * in a dedicated thread, it ensures that the servo positions are updated regularly without blocking the main
 * thread, which can handle other tasks like sensor readings and user input. The class optimizes the
 * control loop by only calculating and updating the servo power if the error is outside of the set threshold,
 * improving efficiency.
 */
public class DrivetrainUpdater extends Thread {
    private final RobotManager robotManager;

    public DrivetrainUpdater(RobotManager robotManager) {
        this.robotManager = robotManager;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("DrivetrainUpdater");
    }

    @Override
    public void run() {
        while (robotManager.isRunning) {
            robotManager.refreshData();
            for (int i = 0; i < robotManager.swerveModules.length; i++) {
                if (robotManager.swerveServosPIDF[i].getError() <= Constants.SWERVE_SERVO_PIDF_TOLERANCE_DEGREES) {
                    Status.swerveServoStatus.put(i, Status.ServoStatus.TARGET_REACHED);
                    robotManager.swerveModules[i].motor.setPower(robotManager.swerveModules[i].motor.targetPower);
                } else {
                    robotManager.swerveModules[i].servo.setPower(robotManager.swerveServosPIDF[i].calculate());
                    Status.swerveServoStatus.put(i, Status.ServoStatus.MOVING);

                    robotManager.swerveModules[i].motor.setPower(
                            robotManager.swerveModules[i].motor.targetPower *
                                    Math.abs(Math.cos(Math.toRadians(robotManager.swerveServosPIDF[i].getError())))
                    );
                }
            }
        }
    }
}
