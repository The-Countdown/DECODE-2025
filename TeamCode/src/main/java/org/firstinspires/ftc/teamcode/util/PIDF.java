package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PIDF {
    private static final Logger log = LoggerFactory.getLogger(PIDF.class);
    // PIDF coefficients
    private double kP,kI,kD, kF;

    /** Limit bound of the output. */
    private double minLimit = Double.NaN ,maxLimit = Double.NaN;
    
    // Dynamic variables
    private double previousTime = Double.NaN;
    private double lastError = 0;
    private double integralError = 0;

    private RobotContainer robotContainer;

    /**
     * Constructs a new PID with set coefficients.
     *
     * @param kP The proportional gain coefficient.
     * @param kI The integral gain coefficient.
     * @param kD The derivative gain coefficient.
     * @param kF The forward gain coefficient.
     */
    public PIDF(RobotContainer robotContainer, final double kP, final double kI, final double kD, final double kF) {
        this.robotContainer = robotContainer;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double update(final double error, final double currentTime) {
    	final double dt = (!Double.isNaN(previousTime)) ? (double)(currentTime - previousTime) : 0;
    	
    	// Compute Integral & Derivative error
        final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        integralError += error * dt;

        robotContainer.telemetry.addData("sk-error", error);
        robotContainer.telemetry.addData("sk-lastError", error);

        // Save history
        previousTime = currentTime;
        lastError = error;

        robotContainer.telemetry.addData("sk-dt", dt);
        robotContainer.telemetry.addData("sk-de", derivativeError);

        robotContainer.telemetry.addData("sk-p", (kP * error));
        robotContainer.telemetry.addData("sk-i", (kI * integralError));
        robotContainer.telemetry.addData("sk-kd", (kD));
        robotContainer.telemetry.addData("sk-d", (kD * derivativeError));
        robotContainer.telemetry.addData("sk-f", (kF * Math.signum(error)));

        double fi;

        if (Double.isNaN(kI * integralError)) {
            fi = 0;
        } else {
            fi = kI * integralError;
        }

        return checkLimits((kP * error) + fi + (kD * derivativeError) + (kF * Math.signum(error)));
    }

    // TODO: Add function updatePIDF that will recreate the PIDF if the current PIDF values are different than the given.
    // But I don't always want to reset.

    public double getPIDFError(final double target, final double currentTime, final double currentValue) {
        return target - currentValue;
    }

    public PIDF updateValues(RobotContainer robotContainer, double kp, double ki, double kd, double kf) {
        if (this.kP != kp || this.kI != ki || this.kD != kd || this.kD != kf) {
            return new PIDF(robotContainer, kp, ki, kd, kf);
        } else {
            return this;
        }
    }
    
    /**
     * Resets the integral and derivative errors.
     */
    public void reset() {
        previousTime = 0;
        lastError = 0;
        integralError = 0;
    }

    /**
     * Bounds the PID output between the lower limit
     * and the upper limit.
     * 
     * @param output The target output value.
     * @return The output value, bounded to the limits.
     */
    private double checkLimits(final double output){
    	if (!Double.isNaN(minLimit) && output < minLimit)
    		return 0;
    	else if (!Double.isNaN(maxLimit) && output > maxLimit)
    		return 0;
    	else if (Double.isNaN(output)) {
            return 0;
        } else {
    		return output;
        }
    }
}
