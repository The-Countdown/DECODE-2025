package org.firstinspires.ftc.teamcode.drivetrain;

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

    /**
     * Constructor for the Drivetrain class.
     * @param robotContainer The Robot object that contains all hardware devices.
     */
    public Drivetrain(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /**
     * This function scales all the powers to ensure they are within the -1 and 1 ranges.
     * This avoids breaking any of the robot functions.
     * @param powers the array of powers for all swerve modules.
     * @return the same array, but with the powers being scaled appropriately.
     */
    public double[] scalePowers(double[] powers) {
        double maxPower = 0;
        // Iterate through each power value in the array.
        for (int i = 0; i < powers.length; i++) {
            // if the power is greater then the current max power then we update it.
            if(powers[i] > maxPower){
                maxPower = powers[i];
            }
        }
        // if the maxPower is above one, that means that we need to scale.
        if(maxPower > 1){
            // iterate through and divide every element by the max power.
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

        // On this edge case the servo will not move. If fixing the problem is less expensive than this, please do so.
        if (normalizedAngle == 90) {
            normalizedAngle = 89.999;
        }
        if (normalizedAngle == -90 || normalizedAngle == -180) {
            normalizedAngle += 0.001;
        }

        return normalizedAngle;
    }

    public void mecanumDrive(double x, double y, double rX, double rT, double lT) {
        double yStickLMulti = 0.8;
        double xStickLMulti = 0.4;
        double xStickRMulti = 0.4;
        double[] powers = new double[4];

        double xStickR;
        double xStickL;
        double yStickL;

        xStickR = rX * (xStickRMulti + (rT * 0.45) - (lT * 0.1));
        xStickL = x * (xStickLMulti + (rT * 0.5) - (lT * 0.2));
        yStickL = y * (-(yStickLMulti + (rT * 0.5) - (lT * 0.2)));

        powers[1] = yStickL + xStickL + xStickR;
        powers[2] = yStickL - xStickL + xStickR;
        powers[0] = yStickL - xStickL - xStickR;
        powers[3] = yStickL + xStickL - xStickR;

        powers = scalePowers(powers);

        RobotContainer.HardwareDevices.driveMotors[1].setPower(powers[1]);
        RobotContainer.HardwareDevices.driveMotors[2].setPower(powers[2]);
        RobotContainer.HardwareDevices.driveMotors[0].setPower(powers[0]);
        RobotContainer.HardwareDevices.driveMotors[3].setPower(powers[3]);
    }

    //RobotContainer.HardwareDevices.driveMotors[3].setPower(yStickL - xStickL + xStickR);
    //RobotContainer.HardwareDevices.driveMotors[2].setPower(yStickL + xStickL + xStickR);
    //RobotContainer.HardwareDevices.driveMotors[0].setPower(yStickL - xStickL - xStickR);
    //RobotContainer.HardwareDevices.driveMotors[1].setPower(yStickL + xStickL - xStickR);

    /**
     * This funciton scales the power of the joystick to follow a curve, so that it allows for finer adjustments.
     * It is clamped between -1 and 1 out of caution, although it doesn't need it, it allows for changes to the curve.
     * @param input
     * @return
     */
    public double joystickScaler(double input) {
        return Math.max(-1, Math.min(1, Math.pow(Math.abs(input), Constants.JOYSTICK_SCALER_EXPONENT) * input));
    }
}
