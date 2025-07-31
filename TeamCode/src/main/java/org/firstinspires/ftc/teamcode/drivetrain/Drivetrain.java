package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.other.PinpointUpdater;

import java.util.Arrays;

/**
 * This class handles the control and calculations for the robot's drivetrain, including swerve drive functionality.
 */
public class Drivetrain extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;

    private static final ElapsedTime stopTimer = new ElapsedTime();

    double[] lastAngles = Constants.SWERVE_STOP_FORMATION;

    /**
     * Constructor for the Drivetrain class.
     * @param robotContainer The Robot object that contains all hardware devices.
     */
    public Drivetrain(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /**
     * This method calculates and sets the target angles and powers for each swerve module based on directional inputs.
     *
     * @param x             The x-axis translational input from -1 to 1.
     * @param y             The y-axis translational input from -1 to 1.
     * @param rX            The rotational input from -1 to 1.
     * @param fieldOriented A boolean indicating whether to use field-oriented driving.
     */
    public void swerveDirectionalInput(double x, double y, double rX, boolean fieldOriented) {
        double rotationalMagnitude = Math.abs(rX);

        if (robotContainer.gamepadEx1.rightStickX.wasJustReleased()) {
            robotContainer.headingPID.setTargetHeading(PinpointUpdater.currentHeading);
        }

        if (rX == 0) {
            rotationalMagnitude = robotContainer.headingPID.calculate(PinpointUpdater.currentHeading);
        }

        if ((robotContainer.gamepadEx1.leftStickX.wasJustReleased() ||
                robotContainer.gamepadEx1.leftStickY.wasJustReleased() ||
                robotContainer.gamepadEx1.rightStickX.wasJustReleased()) &&
                Status.robotHeadingTargetReached && x == 0 && y == 0 && rX == 0) {
            swerveSetTargets(lastAngles, Constants.SWERVE_NO_POWER);
            stopTimer.reset();
            return;
        }

        if (Status.robotHeadingTargetReached && x == 0 && y == 0 && rX == 0 && stopTimer.seconds() >= 1) {
            swerveSetTargets(Constants.SWERVE_STOP_FORMATION, Constants.SWERVE_NO_POWER);
            return;
        }

        if (Status.robotHeadingTargetReached && x == 0 && y == 0 && rX == 0) {
            swerveSetTargets(lastAngles, Constants.SWERVE_NO_POWER);
            return;
        }

        // Determine the rotational direction based on the sign of rX.
        int rotationalDirection = rX >= 0 ? 1 : -1;

        // Calculate the magnitude of translational movement.
        double translationalMagnitude = Math.sqrt(x * x + y * y);
        // Calculate the angle of translational movement.
        double translationalAngle = Math.atan2(y, x);
        // Set the initial translational direction to forward.
        int translationalDirection = 1;

        double currentHeading = normalizeAngle(PinpointUpdater.currentHeading);
        // Adjust the translational angle for field-oriented driving if enabled.
        robotContainer.telemetry.addData("First translationalAngle", translationalAngle);
        translationalAngle = normalizeAngle(Math.toDegrees(translationalAngle));
        robotContainer.telemetry.addData("translationalAngle after first normalize", translationalAngle);
        translationalAngle = fieldOriented ? translationalAngle + currentHeading : translationalAngle;
        robotContainer.telemetry.addData("translationalAngle after subtracting current heading", translationalAngle);
        translationalAngle = normalizeAngle(translationalAngle);
        robotContainer.telemetry.addData("translationalAngle after second normalize", translationalAngle);

        translationalAngle = Math.toRadians(translationalAngle);

        double[] calculatedAngles = new double[robotContainer.swerveModules.length];
        double[] calculatedPowers = new double[robotContainer.swerveModules.length];

        // Iterate through each swerve module to calculate its target angle and power.
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            // Calculate the x and y components of translational movement.
            double translationalX = translationalMagnitude * Math.cos(translationalAngle) * translationalDirection;
            double translationalY = translationalMagnitude * Math.sin(translationalAngle) * translationalDirection;

            // Calculate the x and y components of rotational movement.
            double rotationalX = rotationalMagnitude * Constants.SWERVE_ROTATION_FORMATION_COSINES_RADIANS[i] * rotationalDirection;
            double rotationalY = rotationalMagnitude * Constants.SWERVE_ROTATION_FORMATION_SINES_RADIANS[i] * rotationalDirection;

            // Combine the translational and rotational components into a single vector.
            double vectorX = translationalX + rotationalX;
            double vectorY = translationalY + rotationalY;

            // Calculate the magnitude and angle of the combined vector.
            double vectorMagnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);

            calculatedAngles[i] = Math.toDegrees(vectorAngle);
            calculatedPowers[i] = vectorMagnitude;
        }

        if (!Arrays.stream(calculatedAngles).allMatch(v -> Math.abs(v) <= 0.01)) {
            lastAngles = calculatedAngles;
        }
        swerveSetTargets(calculatedAngles, calculatedPowers);
    }

    /**
     * This method sets the target angles and powers for each swerve module, handling motor inversion if necessary.
     * @param targetAngles An array of target angles for each swerve module.
     * @param targetPowers An array of target powers for each swerve module.
     */
    public void swerveSetTargets(double[] targetAngles, double[] targetPowers) {
        if (!Arrays.stream(targetPowers).allMatch(v -> v == 0)) {
            targetPowers = scalePowers(targetPowers);
        }

        for (int i = 0; i < swerveServos.length; i++) {
            double currentAngle = robotContainer.swerveModules[i].servo.getAngle();
            double error = targetAngles[i] - currentAngle;
            error = normalizeAngle(error);

            if (!Constants.SWERVE_MODULE_FLIPPED[i] && Math.abs(error) > Constants.SWERVE_MODULE_FLIP_SWITCH_ON) {
                Constants.SWERVE_MODULE_FLIPPED[i] = true;
            } else if (Constants.SWERVE_MODULE_FLIPPED[i] && Math.abs(error) <  Constants.SWERVE_MODULE_FLIP_SWITCH_OFF) {
                Constants.SWERVE_MODULE_FLIPPED[i] = false;
            }

            if (Constants.SWERVE_MODULE_FLIPPED[i]) {
                targetAngles[i] = normalizeAngle(targetAngles[i] + 180);
                robotContainer.swerveModules[i].motor.setTargetPower(-targetPowers[i]);
            } else {
                robotContainer.swerveModules[i].motor.setTargetPower(targetPowers[i]);
            }

            robotContainer.swerveModules[i].servo.setTargetAngle(targetAngles[i]);
            Constants.SWERVE_MODULE_FLIPPED[i] = false;
        }
    }

    /**
     * This function scales all the powers to ensure they are within the -1 and 1 ranges.
     * This avoids breaking any of the robot functions.
     * @param powers the array of powers for all swerve modules.
     * @return the same array, but with the powers being scaled appropriately.
     */
    public double[] scalePowers(double[] powers) {
        double maxPower = 0;
        // Find the largest magnitude, not just the largest positive.
        for (int i = 0; i < powers.length; i++) {
            if (Math.abs(powers[i]) > maxPower) {
                maxPower = Math.abs(powers[i]);
            }
        }
        // If any wheel wants more than 100%, scale them all back into [-1,1].
        if (maxPower > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
        return powers;
    }

    /**
     * Normalizes an angle to the range [-180, 180).
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    public double normalizeAngle(double angle) {
        // On this edge case the servo will not move. If fixing the problem is less expensive than this, please do so.
        if (angle == 90) {
            angle = 89.999;
        }
        if (angle == -90 || angle == -180) {
            angle += 0.001;
        }

        // Check if the angle is already in the desired range.
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [-360, 360).
        double normalizedAngle = angle % 360;

        // If the result was negative, shift it to the range [0, 360).
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }

        // If the angle is in the range [180, 360), shift it to [-180, 0).
        if (normalizedAngle >= 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }

    /**
     * This function scales the power of the joystick to follow a curve, so that it allows for finer adjustments.
     * It allows for changes to the curve with the constant JOYSTICK_SCALER_EXPONENT.
     * @param input the input from the joystick
     * @return the scaled power
     */
    public double joystickScaler(double input) {
        return Math.pow(Math.abs(input), Constants.JOYSTICK_SCALER_EXPONENT) * input;
    }
}