package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
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
    private final RobotContainer robotContainer;

    public DrivetrainUpdater(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("DrivetrainUpdater");
    }

    @Override
    public void run() {
        while (robotContainer.isRunning) {
            robotContainer.refreshData();
            for (int i = 0; i < robotContainer.swerveModules.length; i++) {
                if (robotContainer.swerveServosPIDF[i].getError() <= Constants.SWERVE_SERVO_PIDF_TOLERANCE_DEGREES) {
                    Status.swerveServoStatus.put(i, Status.ServoStatus.TARGET_REACHED);
                    robotContainer.swerveModules[i].motor.setPower(robotContainer.swerveModules[i].motor.targetPower);
                } else {
                    robotContainer.swerveModules[i].servo.setPower(robotContainer.swerveServosPIDF[i].calculate());
                    Status.swerveServoStatus.put(i, Status.ServoStatus.MOVING);

                    robotContainer.swerveModules[i].motor.setPower(
                            robotContainer.swerveModules[i].motor.targetPower *
                                    Math.abs(Math.cos(Math.toRadians(robotContainer.swerveServosPIDF[i].getError())))
                    );
                }
            }
        }
    }
}
