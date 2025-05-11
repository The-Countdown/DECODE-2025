package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOp;

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
    private final double[] currentPowers = new double[4];
    private double lastTimestamp = -1;
    private ElapsedTime deltaTimer = new ElapsedTime();

    // power per second
    private static final double maxAccel = 1;

    public DrivetrainUpdater(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("DrivetrainUpdater");
    }

    @Override
    public void run() {
        while (TeleOp.isRunning) {
            robotContainer.refreshData();

            double now = deltaTimer.seconds();
            deltaTimer.reset();
            if (lastTimestamp < 0) {
                lastTimestamp = now;
            }
            double deltaTime = now - lastTimestamp;
            lastTimestamp = now;

            for (int i = 0; i < robotContainer.swerveModules.length; i++) {
                currentPowers[i] = RobotContainer.HardwareDevices.driveMotors[i].getPower();
                double target = robotContainer.swerveModules[i].motor.targetPower;

                double maxDelta = maxAccel * deltaTime;
                double error = target - currentPowers[i];
                double delta = Math.signum(error) * Math.min(Math.abs(error), maxDelta);

                currentPowers[i] += delta;
                double acceleratedMotorPower = currentPowers[i];

                if (robotContainer.swerveServosPIDF[i].getError() <= Constants.SWERVE_SERVO_PIDF_TOLERANCE_DEGREES) {
                    Status.swerveServoStatus.put(i, Status.ServoStatus.TARGET_REACHED);

                    robotContainer.swerveModules[i].motor.setPower(acceleratedMotorPower);
                } else {
                    robotContainer.swerveModules[i].servo.setPower(robotContainer.swerveServosPIDF[i].calculate());
                    Status.swerveServoStatus.put(i, Status.ServoStatus.MOVING);

                    robotContainer.swerveModules[i].motor.setPower(
                            acceleratedMotorPower *
                                    Math.abs(Math.cos(Math.toRadians(robotContainer.swerveServosPIDF[i].getError())))
                    );
                }
            }
        }
    }
}
