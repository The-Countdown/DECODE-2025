package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final CRServoImplEx spindexerServo;
    private final AnalogInput spindexAnalog;
    private static double targetAngle = 0;
    private double error;
    private double lastError;
    private boolean lastSign;
    private double p;
    private double i;
    private double d;
    private double ff;
    private ElapsedTime iTimer;

    public Spindexer (RobotContainer robotContainer, CRServoImplEx spindexerServo, AnalogInput spindexAnalog) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServo;
        this.spindexAnalog = spindexAnalog;
        this.iTimer = new ElapsedTime();
    }

    public void setPower(double power) {
        spindexerServo.setPower(power);
    }

    public void setTargetAngle(double angle) {
        targetAngle = HelperFunctions.normalizeAngle(angle);
    }

    public double getAngle() {
        double angle = (spindexAnalog.getVoltage() / Constants.ANALOG_MAX_VOLTAGE) * 360;

        angle += Constants.SPINDEXER_ANGLE_OFFSET;

        angle = HelperFunctions.normalizeAngle(angle);

        return angle;
    }

    public class PIDF {
        public double getError() {
            double error = targetAngle - robotContainer.spindexer.getAngle();
            error = HelperFunctions.normalizeAngle(error);

            return error;
        }
        public double calculate() {
            // If current sign and last sign are different reset I.
            if (Math.signum(error) > 0) { // Current sign pos
                if (!lastSign) { // Last sign neg
                    iTimer.reset();
                }
            } else if (Math.signum(error) < 0) { // Current sign neg
                if (lastSign) { // Last sign pos
                    iTimer.reset();
                }
            }

            error = getError();

            p = Constants.SPINDEXER_KP * error;
            i = Constants.SPINDEXER_KI * iTimer.milliseconds() * Math.signum(error);
            d = Math.signum(error) * (Constants.SPINDEXER_KD * (lastError - error));
            ff = Constants.SPINDEXER_KF * Math.signum(error);


            if (Math.signum(error) > 0) { // Current sign pos
                lastSign = true;
            } else if (Math.signum(error) < 0) { // Current sign neg
                lastSign = false;
            }

            lastError = error;
            return p + i + ff + d;
        }
    }
}
