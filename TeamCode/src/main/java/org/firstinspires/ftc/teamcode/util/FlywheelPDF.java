package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class FlywheelPDF {
    private final RobotContainer robotContainer;
    private LinkedMotors flywheelMotors;

    private double error;
    private double lastError;
    private double p;
    private double ff;
    private double d;

    public FlywheelPDF(RobotContainer robotContainer, LinkedMotors flywheelMotors) {
        this.robotContainer = robotContainer;
        this.flywheelMotors = flywheelMotors;
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double targetSpeed) {
        double error = Math.abs(targetSpeed - flywheelMotors.getVelocity());

        if (error > 200) {
            return 1;
        }

        p = Constants.Turret.FLYWHEEL_P * error;
        ff = Constants.Turret.FLYWHEEL_F * Math.signum(error);

        d = Math.signum(error) * (Constants.Turret.FLYWHEEL_D * (lastError - error));

        lastError = error;
        return Math.abs(p + d + ff);
    }
}
