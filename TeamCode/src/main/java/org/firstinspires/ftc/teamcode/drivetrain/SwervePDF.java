package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain.ServoStatus;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

/**
 * A PDF controller for a swerve module's servo.
 */
public class SwervePDF {
    private final RobotContainer robotContainer;
    private final int module;
    private double targetAngle;

    private boolean lastSign;
    private double error;
    private double lastError;
    private double p;
    private ElapsedTime iTimer;
    private double swerveConstantPower;
    private double ff;
    private double d;

    public SwervePDF(RobotContainer robotContainer, int module) {
        this.robotContainer = robotContainer;
        this.module = module;
        this.targetAngle = Constants.Swerve.STOP_FORMATION[module];
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
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate() {
        if (robotContainer.drivetrain.swerveServoStatus[module] == ServoStatus.TARGET_REACHED) {
            iTimer.reset();
        }

        error = getError();

        p = Constants.Swerve.SERVO_KP[module] * error;
        ff = Constants.Swerve.SERVO_KF[module] * Math.signum(error);

        d = Math.signum(error) * (Constants.Swerve.SERVO_KD[module] * (lastError - error));

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
        return p + d + ff;
    }
}
