package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SwerveModule {
    private Robot robot;
    private DcMotorEx drivingMotor;
    private CRServoImplEx turningServo;
    private AnalogInput analogEncoder;
    private int module;
    private SwerveServoPIDF servoPIDF;

    public SwerveModule(Robot robot, DcMotorEx motor, CRServoImplEx turningServo, AnalogInput analogEncoder, int module) {
        this.robot = robot;
        this.drivingMotor = motor;
        this.turningServo = turningServo;
        this.analogEncoder = analogEncoder;
        this.module = module;
        this.servoPIDF = new SwerveServoPIDF(robot, module, turningServo);
    }

    public class Servo {
        //Gets the position in degrees of the swerve module servo ranging from -180 to 180, 0 being forwards
        public double getAngle() {
            robot.refreshData();
            double angle = analogEncoder.getVoltage() / ((Constants.ANALOG_MAX_VOLTAGE * 360) + Constants.SWERVE_SERVO_ANGLE_OFFSET[4] + Constants.SWERVE_SERVO_ANGLE_OFFSET[module]);

            if (angle > 180) {
                angle -= 360;
            }

            return angle;
        }

        public void setAngle(double angle) {
            servoPIDF.setTargetAngle(angle);
        }
    }

    public class Motor {
      public void setMotorPower(double power) {
          drivingMotor.setPower(power);
      }
    }
    Servo servo = new Servo();
}
