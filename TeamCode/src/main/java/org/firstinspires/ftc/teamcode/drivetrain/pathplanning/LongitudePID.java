package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

/**
 * A PID controller for maintaining a specific LONGITUDE.
 */
public class LongitudePID {
    private final RobotContainer robotContainer;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double currentTime;
    private double lastTime;
    private double p;
    private double i;
    private double d;

    public LongitudePID(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public double getError(){
       return lastError;
    }
    /**
     * Calculates the PID output for the servo.
     *
     * @return The calculated PID output.
     */
    public double calculate() {
        double error = Status.targetPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
        currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        if (Math.abs(error) < Constants.Pathing.LONGITUDE_PID_TOLERANCE_CM) {
            lastError = error;
            Status.robotLongitudeTargetReached = true;
            return 0;
        } else {
            Status.robotLongitudeTargetReached = false;
        }

        if (dt < 1e-4) dt = 1e-4;  // safety clamp

        p = Constants.Pathing.LONGITUDE_KP * error;

        i += Constants.Pathing.LONGITUDE_KI * error * currentTime;
        i = Math.max(-Constants.Pathing.LONGITUDE_I_MAX, Math.min(Constants.Pathing.LONGITUDE_I_MAX, i));

        d = -Constants.Pathing.LONGITUDE_KD * (error - lastError) / currentTime;

        lastError = error;

        return p + i + d + (Constants.Pathing.LONGITUDE_KF  * Math.signum(error));
    }
}
