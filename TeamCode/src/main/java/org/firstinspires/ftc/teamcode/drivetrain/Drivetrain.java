package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.Arrays;

/**
 * This class handles the control and calculations for the robot's drivetrain, including swerve drive functionality.
 */
public class Drivetrain extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private static final ElapsedTime stopTimer = new ElapsedTime();
    double[] calculatedAngles;
    double[] calculatedPowers;
    double[] lastAngles = Constants.Swerve.STOP_FORMATION;
    GamepadWrapper.ButtonReader xButton = new GamepadWrapper.ButtonReader();
    GamepadWrapper.ButtonReader yButton = new GamepadWrapper.ButtonReader();
    GamepadWrapper.ButtonReader rXButton = new GamepadWrapper.ButtonReader();

    /**
     * Constructor for the Drivetrain class.
     * @param robotContainer The Robot object that contains all hardware devices.
     */
    public Drivetrain(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.calculatedAngles = new double[Constants.Swerve.NUM_SERVOS];
        this.calculatedPowers = new double[Constants.Swerve.NUM_MOTORS];
    }

    /**
     * This method calculates and sets the target angles and powers for each swerve module based on directional inputs.
     *
     */
    public void controlUpdate() {
        if (!Status.isDrivingActive) {
            return;
        }

        double x = joystickScaler(-robotContainer.gamepadEx1.leftStickX());
        double y = joystickScaler(robotContainer.gamepadEx1.leftStickY());
        double rX = joystickScaler(-robotContainer.gamepadEx1.rightStickX());
        double rotationalMagnitude = Math.abs(rX);

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            Status.fieldOriented = !Status.fieldOriented;
        }

        if ((robotContainer.gamepadEx1.leftStickX.wasJustReleased() ||
                robotContainer.gamepadEx1.leftStickY.wasJustReleased() ||
                robotContainer.gamepadEx1.rightStickX.wasJustReleased()) &&
                x == 0 && y == 0 && rX == 0) {
            Status.flywheelToggle = true;
            setTargets(lastAngles, Constants.Swerve.NO_POWER);
            stopTimer.reset();
            return;
        }

        if (x == 0 && y == 0 && rX == 0 && stopTimer.seconds() >= 1) {
            Status.flywheelToggle = true;
            setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
            return;
        }

        if ( x == 0 && y == 0 && rX == 0) {
            Status.flywheelToggle = true;
            setTargets(lastAngles, Constants.Swerve.NO_POWER);
            return;
        }

//        Status.flywheelToggle = false;

        // Determine the rotational direction based on the sign of rX.
        int rotationalDirection = rX >= 0 ? 1 : -1;

        // Calculate the magnitude of translational movement.
        double translationalMagnitude = Math.sqrt(x * x + y * y);
        // Calculate the angle of translational movement.
        double translationalAngle = Math.atan2(y, x);

        double currentHeading = HelperFunctions.normalizeAngle(Status.currentHeading + (Status.alliance == Constants.Game.ALLIANCE.RED ? 90 : -90));
        // Adjust the translational angle for field-oriented driving if enabled.
        translationalAngle = Status.fieldOriented ? translationalAngle + Math.toRadians(currentHeading) : translationalAngle;

        // Calculate the x and y components of translational movement.
        double translationalX = translationalMagnitude * Math.cos(translationalAngle);
        double translationalY = translationalMagnitude * Math.sin(translationalAngle);

        // Iterate through each swerve module to calculate its target angle and power.
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            // Calculate the x and y components of rotational movement.
            double rotationalX = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_COSINES_RADIANS[i] * rotationalDirection;
            double rotationalY = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_SINES_RADIANS[i] * rotationalDirection;

            // Combine the translational and rotational components into a single vector.
            double vectorX = translationalX + rotationalX;
            double vectorY = translationalY + rotationalY;

            // Calculate the magnitude and angle of the combined vector.
            double vectorMagnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);

            calculatedAngles[i] = Math.toDegrees(vectorAngle);
            calculatedPowers[i] = vectorMagnitude;
        }

        boolean anyNonZero = false;
        for (double v : calculatedAngles) {
            if (Math.abs(v) > 0.01) {
                anyNonZero = true;
                break;
            }
        }
        if (anyNonZero) {
            lastAngles = calculatedAngles;
        }

        setTargets(calculatedAngles, calculatedPowers);
    }

    public void powerInput(double x, double y, double rX) {
        xButton.update(x != 0);
        yButton.update(y != 0);
        rXButton.update(rX != 0);

        if ((xButton.wasJustReleased() ||
                yButton.wasJustReleased() ||
                rXButton.wasJustReleased()) &&
                x == 0 && y == 0 && rX == 0) {
//            Status.flywheelToggle = true;
            setTargets(lastAngles, Constants.Swerve.NO_POWER);
            stopTimer.reset();
            return;
        }

        if (x == 0 && y == 0 && rX == 0 && stopTimer.seconds() >= 1) {
//            Status.flywheelToggle = true;
            setTargets(Constants.Swerve.STOP_FORMATION, Constants.Swerve.NO_POWER);
            return;
        }

        if (x == 0 && y == 0 && rX == 0) {
//            Status.flywheelToggle = true;
            setTargets(lastAngles, Constants.Swerve.NO_POWER);
            return;
        }
//
//        Status.flywheelToggle = false;

        double rotationalMagnitude = Math.abs(rX);
        // Determine the rotational direction based on the sign of rX.
        int rotationalDirection = rX >= 0 ? 1 : -1;

        // Calculate the magnitude of translational movement.
        double translationalMagnitude = Math.sqrt(x * x + y * y);
        // Calculate the angle of translational movement.
        double translationalAngle = Math.atan2(y, x);

        double currentHeading = HelperFunctions.normalizeAngle(Status.currentHeading);
        translationalAngle += Math.toRadians(currentHeading);

        // Calculate the x and y components of translational movement.
        double translationalX = translationalMagnitude * Math.cos(translationalAngle);
        double translationalY = translationalMagnitude * Math.sin(translationalAngle);

        // Iterate through each swerve module to calculate its target angle and power.
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            // Calculate the x and y components of rotational movement.
            double rotationalX = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_COSINES_RADIANS[i] * rotationalDirection;
            double rotationalY = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_SINES_RADIANS[i] * rotationalDirection;

            // Combine the translational and rotational components into a single vector.
            double vectorX = translationalX + rotationalX;
            double vectorY = translationalY + rotationalY;

            // Calculate the magnitude and angle of the combined vector.
            double vectorMagnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);

            calculatedAngles[i] = Math.toDegrees(vectorAngle);
            calculatedPowers[i] = vectorMagnitude;
        }

        boolean anyNonZero = false;
        for (double v : calculatedAngles) {
            if (Math.abs(v) > 0.01) {
                anyNonZero = true;
                break;
            }
        }
        if (anyNonZero) {
            lastAngles = calculatedAngles;
        }

        setTargets(calculatedAngles, calculatedPowers);
    }

    public double[] fakePowerInput(double x, double y, double rX) {
        xButton.update(x != 0);
        yButton.update(y != 0);
        rXButton.update(rX != 0);

        if (x == 0 && y == 0 && rX == 0) {
            return new double[] {0.0, 0.0};
        }

        double rotationalMagnitude = Math.abs(rX);
        // Determine the rotational direction based on the sign of rX.
        int rotationalDirection = rX >= 0 ? 1 : -1;

        // Calculate the magnitude of translational movement.
        double translationalMagnitude = Math.sqrt(x * x + y * y);
        // Calculate the angle of translational movement.
        double translationalAngle = Math.atan2(y, x);

        double currentHeading = HelperFunctions.normalizeAngle(Status.currentHeading);
        translationalAngle += Math.toRadians(currentHeading);

        // Calculate the x and y components of translational movement.
        double translationalX = translationalMagnitude * Math.cos(translationalAngle);
        double translationalY = translationalMagnitude * Math.sin(translationalAngle);

        // Iterate through each swerve module to calculate its target angle and power.
        for (int i = 0; i < robotContainer.swerveModules.length; i++) {
            // Calculate the x and y components of rotational movement.
            double rotationalX = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_COSINES_RADIANS[i] * rotationalDirection;
            double rotationalY = rotationalMagnitude * Constants.Swerve.ROTATION_FORMATION_SINES_RADIANS[i] * rotationalDirection;

            // Combine the translational and rotational components into a single vector.
            double vectorX = translationalX + rotationalX;
            double vectorY = translationalY + rotationalY;

            // Calculate the magnitude and angle of the combined vector.
            double vectorMagnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);

            calculatedAngles[i] = Math.toDegrees(vectorAngle);
            calculatedPowers[i] = vectorMagnitude;
        }

        boolean anyNonZero = false;
        for (double v : calculatedAngles) {
            if (Math.abs(v) > 0.01) {
                anyNonZero = true;
                break;
            }
        }
        if (anyNonZero) {
            lastAngles = calculatedAngles;
        }

        return new double[] {calculatedAngles[0], calculatedPowers[0]};
    }

    /**
     * This method sets the target angles and powers for each swerve module, handling motor inversion if necessary.
     * @param targetAngles An array of target angles for each swerve module.
     * @param targetPowers An array of target powers for each swerve module.
     */
    public void setTargets(double[] targetAngles, double[] targetPowers) {
        if (!Arrays.stream(targetPowers).allMatch(v -> v == 0)) {
            targetPowers = scalePowers(targetPowers);
        }

        for (int i = 0; i < swerveServos.length; i++) {
            double currentAngle = robotContainer.swerveModules[i].servo.getAngle();
            double error = targetAngles[i] - currentAngle;
            error = HelperFunctions.normalizeAngle(error);

            if (!Constants.Swerve.MODULE_FLIPPED[i] && Math.abs(error) > Constants.Swerve.MODULE_FLIP_SWITCH_ON) {
                Constants.Swerve.MODULE_FLIPPED[i] = true;
            } else if (Constants.Swerve.MODULE_FLIPPED[i] && Math.abs(error) <  Constants.Swerve.MODULE_FLIP_SWITCH_OFF) {
                Constants.Swerve.MODULE_FLIPPED[i] = false;
            }

            if (Constants.Swerve.MODULE_FLIPPED[i]) {
                targetAngles[i] = HelperFunctions.normalizeAngle(targetAngles[i] + 180);
                robotContainer.swerveModules[i].motor.setTargetPower(-targetPowers[i]);
            } else {
                robotContainer.swerveModules[i].motor.setTargetPower(targetPowers[i]);
            }

            robotContainer.swerveModules[i].servo.setTargetAngle(targetAngles[i]);
            Constants.Swerve.MODULE_FLIPPED[i] = false;
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
     * This function scales the power of the joystick to follow a curve, so that it allows for finer adjustments.
     * It allows for changes to the curve with the constant JOYSTICK_SCALER_EXPONENT.
     * @param input the input from the joystick
     * @return the scaled power
     */
    public double joystickScaler(double input) {
        return Math.pow(Math.abs(input), Constants.Control.JOYSTICK_SCALER_EXPONENT) * input;
    }
}
