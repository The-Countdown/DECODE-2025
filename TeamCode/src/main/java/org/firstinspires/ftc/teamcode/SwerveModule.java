package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Represents a single Swerve Drive Module, encapsulating its motor and servo controls, as well as sensor data.
 */
public class SwerveModule {
    private final Robot robot;
    private final DcMotorEx drivingMotor;
    private final CRServoImplEx turningServo;
    private final AnalogInput analogEncoder;
    private final int moduleIndex;
    private final SwerveServoPIDF servoPIDF;

    /**
     * Constructor for the SwerveModule class.
     *
     * @param robot         The Robot instance.
     * @param motor         The driving motor of the module.
     * @param turningServo  The servo responsible for turning the module.
     * @param analogEncoder The analog encoder for reading the module's angle.
     * @param moduleIndex   The index of the module (e.g., 0 for front-left, 1 for front-right, etc.).
     */
    public SwerveModule(Robot robot, DcMotorEx motor, CRServoImplEx turningServo, AnalogInput analogEncoder, int moduleIndex) {
        this.robot = robot;
        this.drivingMotor = motor;
        this.turningServo = turningServo;
        this.analogEncoder = analogEncoder;
        this.moduleIndex = moduleIndex;
        this.servoPIDF = new SwerveServoPIDF(robot, moduleIndex, turningServo);
    }

    public class Servo {

        /**
         * Gets the current angle of the module using the Axon analog encoder and converting the voltage to degrees.
         * Adds an offset to the angle based on the module's index.
         * @return The current angle of the module in degrees, normalized at -180 to 180.
         */
        public double getAngle() {
            double angle = analogEncoder.getVoltage() / ((Constants.ANALOG_MAX_VOLTAGE * 360) + Constants.SWERVE_SERVO_ANGLE_OFFSET[4] + Constants.SWERVE_SERVO_ANGLE_OFFSET[moduleIndex]);

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
      public void setPower(double power) {
          drivingMotor.setPower(power);
      }
    }

    public final Servo servo = new Servo();
    public final Motor motor = new Motor();
}
