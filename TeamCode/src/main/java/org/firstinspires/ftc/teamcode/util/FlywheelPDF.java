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
    private double lastTargetPower = 0;

    public FlywheelPDF(RobotContainer robotContainer, LinkedMotors flywheelMotors) {
        this.robotContainer = robotContainer;
        this.flywheelMotors = flywheelMotors;
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double targetSpeed) {
        if (targetSpeed == 0) {
            lastTargetPower = 0;
            return 0;
        }
        double error = targetSpeed - flywheelMotors.getVelocity();

        robotContainer.telemetry.addData("Flywheel Error", error);

        p = Constants.Turret.FLYWHEEL_P * error;
//        ff = Constants.Turret.FLYWHEEL_F * Math.signum(error);

//        d = Math.signum(error) * (Constants.Turret.FLYWHEEL_D * (lastError - error));

        lastTargetPower = lastTargetPower + p;
        if (lastTargetPower > 1) {
            lastTargetPower = 1;
        }
        return lastTargetPower;
    }
}
