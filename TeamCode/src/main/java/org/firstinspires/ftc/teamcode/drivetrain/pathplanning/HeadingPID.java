package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

/**
 * A PID controller for maintaining a specific heading.
 */
public class HeadingPID {
    private final RobotContainer robotContainer;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double currentTime;
    private double lastTime;
    private double p;
    private double i;
    private double d;

    public HeadingPID(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /**
     * Calculates the PID output for the servo.
     *
     * @return The calculated PID output.
     */
    public double calculate() {
        double error = HelperFunctions.normalizeAngle(Status.targetPose.getHeading(AngleUnit.DEGREES) - Status.currentPose.getHeading(AngleUnit.DEGREES));
        currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        timer.reset();
        if (currentTime < 1e-6) currentTime = 1e-6;

        if (Math.abs(error) < Constants.Pathing.HEADING_PID_TOLERANCE_DEGREES) {
            lastError = error;
            Status.robotHeadingTargetReached = true;
            return 0;
        } else {
            Status.robotHeadingTargetReached = false;
        }

        if (dt < 1e-4) dt = 1e-4;  // safety clamp

        p = Constants.Pathing.HEADING_KP * error;

        i += Constants.Pathing.HEADING_KI * error * currentTime;
        i = Math.max(-Constants.Pathing.HEADING_I_MAX, Math.min(Constants.Pathing.HEADING_I_MAX, i));

        d = -Constants.Pathing.HEADING_KD * (error - lastError) / currentTime;

        lastError = error;

        return p + i + d + (Constants.Pathing.HEADING_KF  * Math.signum(error));
    }
}
