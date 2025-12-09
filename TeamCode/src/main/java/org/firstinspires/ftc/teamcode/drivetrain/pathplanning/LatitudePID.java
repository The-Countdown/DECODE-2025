package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

/**
 * A PID controller for maintaining a specific latitude.
 */
public class LatitudePID {
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError;
    private double currentTime;
    private double lastTime;
    private double p;
    private double i;
    private double d;

    public LatitudePID() {
       this.lastError = 0;
       this.currentTime = 0;
       this.lastTime = 0;
    }

    /**
     * Calculates the PID output for the servo.
     *
     * @return The calculated PID output.
     */
    public double calculate() {
        double error = Status.targetPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        if (Math.abs(error) < Constants.Pathing.LATITUDE_PID_TOLERANCE_CM) {
            lastError = error;
            Status.robotLatitudeTargetReached = true;
            return 0;
        } else {
            Status.robotLatitudeTargetReached = false;
        }

        if (dt < 1e-4) dt = 1e-4;  // safety clamp

        p = Constants.Pathing.LATITUDE_KP * error;

        i += Constants.Pathing.LATITUDE_KI * error * currentTime;
        i = Math.max(-Constants.Pathing.LATITUDE_I_MAX, Math.min(Constants.Pathing.LATITUDE_I_MAX, i));

        d = -Constants.Pathing.LATITUDE_KD * (error - lastError) / currentTime;

        lastError = error;

        return p + i + d + (Constants.Pathing.LATITUDE_KF  * Math.signum(error));
    }
}
