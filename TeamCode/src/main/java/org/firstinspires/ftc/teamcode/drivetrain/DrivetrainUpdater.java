package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final double[] currentPowers = new double[4];
    private final ElapsedTime deltaTimer = new ElapsedTime();

    public DrivetrainUpdater(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("DrivetrainUpdater");
    }

    @Override
    public void run() {
        while (!Status.opModeIsActive);
        deltaTimer.reset();
        robotContainer.refreshData();
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            currentPowers[i] = RobotContainer.HardwareDevices.swerveMotors[i].getPower();
        }
        while (Status.opModeIsActive) {
            robotContainer.refreshData();

            double deltaTime = deltaTimer.seconds();
            deltaTimer.reset();

            for (int i = 0; i < robotContainer.swerveModules.length; i++) {
                double target = robotContainer.swerveModules[i].motor.targetPower;

                double maxDelta = Constants.MAX_DRIVE_ACCELERATION * deltaTime;
                double error = target - currentPowers[i];

                if (!(error < 0)) {
                    double delta = Math.signum(error) * Math.min(Math.abs(error), maxDelta);
                    currentPowers[i] += delta;
                } else {
                    currentPowers[i] = target;
                }

                double acceleratedMotorPower = currentPowers[i];

                if (Math.abs(robotContainer.swerveServosPIDF[i].getError()) <= Constants.SWERVE_SERVO_PIDF_TOLERANCE_DEGREES) {
                    Status.swerveServoStatus.put(i, Status.ServoStatus.TARGET_REACHED);
                    robotContainer.swerveModules[i].servo.setPower(0);

                    robotContainer.swerveModules[i].motor.setPower(acceleratedMotorPower);
                } else {
                    robotContainer.swerveModules[i].servo.setPower(-robotContainer.swerveServosPIDF[i].calculate());
                    Status.swerveServoStatus.put(i, Status.ServoStatus.MOVING);

                    robotContainer.swerveModules[i].motor.setPower(acceleratedMotorPower * Math.abs(Math.cos(Math.toRadians(robotContainer.swerveServosPIDF[i].getError()))));
                }
            }
        }
    }
}
