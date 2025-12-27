package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class PIDF {
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

    /**
     * Updates the controller with the current time and value
     * and outputs the PIDF controller output.
     *
     * @param currentTime The current time (in arbitrary time unit, such as seconds).
     * If the PID is assumed to run at a constant frequency, you can simply put '1'.
     * @param currentValue The current, measured value.
     *
     * @return The PIDF controller output.
     */
    public double update(final double target, final double currentTime, final double currentValue) {
    	final double error = target - currentValue;
    	final double dt = (previousTime != Double.NaN) ? (double)(currentTime - previousTime) : 0;
    	
    	// Compute Integral & Derivative error
        final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        integralError += error * dt;
        
        // Save history
        previousTime = currentTime;
        lastError = error;

        robotContainer.telemetry.addData("sk-p", (kP * error));
        robotContainer.telemetry.addData("sk-i", (kI * integralError));
        robotContainer.telemetry.addData("sk-d", (kD * derivativeError));
        robotContainer.telemetry.addData("sk-f", (kF * Math.signum(error)));

        return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError) + (kF * Math.signum(error)));
    }

    public double getPIDFError(final double target, final double currentTime, final double currentValue) {
        return target - currentValue;
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
            robotContainer.telemetry.addData("NaN", true);
            return 0;
        } else {
    		return output;
        }
    }
}
