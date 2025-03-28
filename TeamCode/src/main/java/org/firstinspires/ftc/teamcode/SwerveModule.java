package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SwerveModule {
    private final Robot robot;
    private final DcMotorEx drivingMotor;
    private final CRServoImplEx turningServo;
    private final AnalogInput analogEncoder;
    private final int module;
    private final SwerveServoPIDF servoPIDF;

    public SwerveModule(Robot robot, DcMotorEx motor, CRServoImplEx turningServo, AnalogInput analogEncoder, int module) {
        this.robot = robot;
        this.drivingMotor = motor;
        this.turningServo = turningServo;
        this.analogEncoder = analogEncoder;
        this.module = module;
        this.servoPIDF = new SwerveServoPIDF(robot, module, turningServo);
    }

    public class Servo {
        // Gets the position in degrees of the swerve module servo ranging from -180 to 180, 0 being forwards
        public double getAngle() {
            double angle = analogEncoder.getVoltage() / ((Constants.ANALOG_MAX_VOLTAGE * 360) + Constants.SWERVE_SERVO_ANGLE_OFFSET[4] + Constants.SWERVE_SERVO_ANGLE_OFFSET[module]);

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
