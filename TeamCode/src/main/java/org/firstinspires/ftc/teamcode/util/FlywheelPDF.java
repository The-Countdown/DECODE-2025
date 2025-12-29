package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class FlywheelPDF {
    private final RobotContainer robotContainer;
    private LinkedMotors flywheelMotors;

    private double error;
    private double lastError;
    private double p;
    private double ff;
    private double d;
    private double lastTargetPower;

    public FlywheelPDF(RobotContainer robotContainer, LinkedMotors flywheelMotors) {
        this.robotContainer = robotContainer;
        this.flywheelMotors = flywheelMotors;
        this.lastTargetPower = 0;
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

        if (error > Constants.Turret.FLYWHEEL_MAX_POWER_ERROR) {
            lastTargetPower = 1;
            return 1;
        }

        robotContainer.telemetry.addData("Flywheel Error", error);

        p = Constants.Turret.FLYWHEEL_P * error;
        //ff = Constants.Turret.FLYWHEEL_F * Math.signum(error);

        d = (Constants.Turret.FLYWHEEL_D * (lastError - error));

        lastTargetPower = lastTargetPower + (p - d);
        if (lastTargetPower > 1) {
            lastTargetPower = 1;
        }
        lastError = error;
        return lastTargetPower  * 1 + ((14 - robotContainer.getVoltage(Constants.Robot.CONTROL_HUB_INDEX)) / 14);
    }
}
