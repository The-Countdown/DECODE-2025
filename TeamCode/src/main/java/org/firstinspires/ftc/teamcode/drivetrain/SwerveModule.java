package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.hardware.BetterCRServo;
import org.firstinspires.ftc.teamcode.hardware.BetterAnalogInput;

/**
 * Represents a single Swerve Drive Module, encapsulating its motor and servo controls, as well as sensor data.
 *
 * <pre>
 *      Quadrant 1    |   Quadrant 0
 *                    |
 *      ------------------------
 *                    |
 *      Quadrant 2    |   Quadrant 3
 *</pre>
 *
 *  Like a coordinate plane, the first quadrant is 0, the second is 1, etc.
 *  This is relating to the positions of the modules on the robot
 * <p>
 *  When indexing from ANY ARRAY IN THIS CODEBASE:
 *  0 - front right, 1 - front left, 2 - back left, 3 - back right
 */
public class SwerveModule {
    private final RobotContainer robotContainer;
    private final BetterDcMotor drivingMotor;
    private final BetterCRServo turningServo;
    private final BetterAnalogInput analogEncoder;
    private final double powerMultiplier;
    public final int moduleIndex;
    private final SwervePDF servoPDF;

    /**
     * Constructor for the SwerveModulePosition class.
     *
     * @param robotContainer         The Robot instance.
     * @param motor         The driving motor of the module.
     * @param turningServo  The servo responsible for turning the module.
     * @param analogEncoder The analog encoder for reading the module's angle.
     * @param powerMultiplier A multiplier to maintain constant velocity for all modules despite differences in friction between modules.
     * @param moduleIndex   The index of the module.
     */
    public SwerveModule(RobotContainer robotContainer, BetterDcMotor motor, BetterCRServo turningServo, SwervePDF servoPDF, BetterAnalogInput analogEncoder, double powerMultiplier, int moduleIndex) {
        this.robotContainer = robotContainer;
        this.drivingMotor = motor;
        this.turningServo = turningServo;
        this.servoPDF = servoPDF;
        this.analogEncoder = analogEncoder;
        this.powerMultiplier = powerMultiplier;
        this.moduleIndex = moduleIndex;
    }

    // I hate getters and setters but I want to keep powerMultiplier private and not mutable by any part of the program.
    public double getPowerMultiplier() {
        return this.powerMultiplier;
    }

    public class Servo {

        /**
         * Gets the current angle of the module using the Axon analog encoder and converting the voltage to degrees.
         * Adds an offset to the angle based on the module's index.
         * @return The current angle of the module in degrees, normalized at -180 to 180.
         */
        public double getAngle() {
            if (!Constants.Swerve.SERVO_ANALOG_ACTIVE) {
                return 0;
            }

            double angle = (analogEncoder.updateGetVoltage() / Constants.System.ANALOG_MAX_VOLTAGE) * 360;

            angle += Constants.Swerve.SERVO_ANGLE_OFFSET[moduleIndex];

            angle = HelperFunctions.normalizeAngle(angle);

            return angle;
        }

        public void setTargetAngle(double angle) {
            servoPDF.setTargetAngle(angle);
        }

        public void setPower(double power) {
            turningServo.updateSetPower(power);
        }
    }

    public class Motor {
        public double targetPower;
        public void setPower(double power) {
            drivingMotor.updateSetPower(power);
        }

        /**
        * Sets the velocity of the motor from 0-1, because it is specific to this swerve motor so the value will be consistent with the multiplier.
         */
        public void setVelocity(double velocity) {
            drivingMotor.updateSetVelocity(velocity * Constants.Swerve.MOTOR_MAX_VELOCITY_TICKS_PER_SECOND);
        }

        public double getVelocity() {
            return drivingMotor.getVelocity();
        }

        // This function takes in a double between 0-1 for 0 rpm to max rpm of motor as relative to the max forward speed of the drive base.
        public void setPowerWithMultiplier(double speed) {
            drivingMotor.updateSetPower(speed * powerMultiplier);
        }

        public void setTargetPower(double power) {
            targetPower = power;
        }
    }

    public final Servo servo = new Servo();
    public final Motor motor = new Motor();
}
