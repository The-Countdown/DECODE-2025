package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

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
    private final DcMotorEx drivingMotor;
    private final CRServoImplEx turningServo;
    private final AnalogInput analogEncoder;
    private final int moduleIndex;
    private final SwervePIDF servoPIDF;

    /**
     * Constructor for the SwerveModulePosition class.
     *
     * @param robotContainer         The Robot instance.
     * @param motor         The driving motor of the module.
     * @param turningServo  The servo responsible for turning the module.
     * @param analogEncoder The analog encoder for reading the module's angle.
     * @param moduleIndex   The index of the module.
     */
    public SwerveModule(RobotContainer robotContainer, DcMotorEx motor, CRServoImplEx turningServo, SwervePIDF servoPIDF, AnalogInput analogEncoder, int moduleIndex) {
        this.robotContainer = robotContainer;
        this.drivingMotor = motor;
        this.turningServo = turningServo;
        this.servoPIDF = servoPIDF;
        this.analogEncoder = analogEncoder;
        this.moduleIndex = moduleIndex;
    }

    public class Servo {

        /**
         * Gets the current angle of the module using the Axon analog encoder and converting the voltage to degrees.
         * Adds an offset to the angle based on the module's index.
         * @return The current angle of the module in degrees, normalized at -180 to 180.
         */
        public double getAngle() {
            if (!Constants.SERVO_ANALOG_ACTIVE) {
                return 0;
            }

            double angle = analogEncoder.getVoltage() / Constants.ANALOG_MAX_VOLTAGE * 360;

            angle += Constants.SWERVE_SERVO_ANGLE_OFFSET[4] + Constants.SWERVE_SERVO_ANGLE_OFFSET[moduleIndex];

            if (angle > 180) {
                angle -= 360;
            }

            return angle;
        }

        public void setTargetAngle(double angle) {
            servoPIDF.setTargetAngle(angle);
        }

        public void setPower(double power) {
            turningServo.setPower(power);
        }
    }

    public class Motor {
        public double targetPower;
        public void setPower(double power) {
            drivingMotor.setPower(power);
        }

        /**
        * Sets the velocity of the motor from 0-1, because it is specific to this swerve motor so the value will be consistent with the multiplier
         */
        public void setVelocity(double velocity) {
            drivingMotor.setVelocity(velocity * Constants.SWERVE_MOTOR_MAX_VELOCITY_TICKS_PER_SECOND);
        }

        public void setTargetPower(double power) {
            targetPower = power;
        }
    }

    public final Servo servo = new Servo();
    public final Motor motor = new Motor();
}
