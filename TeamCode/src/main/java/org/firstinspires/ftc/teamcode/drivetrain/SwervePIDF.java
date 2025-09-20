package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

/**
 * A PIDF controller for a swerve module's servo.
 */
public class SwervePIDF {
    private final RobotContainer robotContainer;
    private final int module;
    private double targetAngle;

    private boolean lastSign;
    private double error;
    private double lastError;
    private double p;
    private double i;
    private ElapsedTime iTimer;
    private double swerveConstantPower;
    private double ff;
    private double d;

    public SwervePIDF(RobotContainer robotContainer, int module) {
        this.robotContainer = robotContainer;
        this.module = module;
        this.targetAngle = Constants.SWERVE_STOP_FORMATION[module];
        this.iTimer = new ElapsedTime();
    }

    public void setTargetAngle(double angle) {
        targetAngle = HelperFunctions.normalizeAngle(angle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        double error = targetAngle - robotContainer.swerveModules[module].servo.getAngle();
        error = HelperFunctions.normalizeAngle(error);

        return error;
    }

    /**
     * Calculates the PIDF output for the servo.
     * @return The calculated PIDF output.
     */
    public double calculate() {
        if (Status.swerveServoStatus[module] == Status.ServoStatus.TARGET_REACHED) {
            iTimer.reset();
        }

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

        p = Constants.SWERVE_SERVO_KP[module] * error;
        i = Constants.SWERVE_SERVO_KI[module] * iTimer.milliseconds() * Math.signum(error);
        // ff = Constants.SWERVE_SERVO_KF[module] * Math.signum(error);

        swerveConstantPower = (Constants.SWERVE_SERVO_KF[module] * (1 - (robotContainer.swerveModules[module].motor.targetPower * Constants.SWERVE_SERVO_MOTOR_FACTOR[module])));
        if (swerveConstantPower < 0) {
            swerveConstantPower = 0;
        }
        ff = swerveConstantPower * Math.signum(error);
        d = Math.signum(error) * (Constants.SWERVE_SERVO_KD[module] * (lastError - error));

//        robotContainer.telemetry.addData("p " + module + " : " , p);
//        robotContainer.telemetry.addData("i " + module + " : ", i);
//        robotContainer.telemetry.addData("ff " + module + " : ", ff);
//        robotContainer.telemetry.addData("d " + module + " : ", d);

        if (Math.signum(error) > 0) { // Current sign pos
            lastSign = true;
        } else if (Math.signum(error) < 0) { // Current sign neg
            lastSign = false;
        }

        lastError = error;
        return p + i + ff + d;
    }
}
