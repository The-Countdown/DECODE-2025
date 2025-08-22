package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

public class Pose2d {
    private double x;
    private double y;
    private double heading;

    //current pose2d
    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    //set pose2d
    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    //straight distance between 2 poses, ignore heading
    public double distanceTo(Pose2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        //triangle therom(forgot name) to get distance
        return Math.sqrt(dx * dx + dy * dy);
    }

    //finds the angle between poses (how much it needs to roatate to hit target)
    public double HeadingError(double currentHeading, double targetHeading) {
        //change in angle + pi modulo * 2pi - math pi
        //+pi makes number positive so its not between -pi and pi and instead 0-2pi, -pi at the end changes it back
        return normalizeAngle(targetHeading - currentHeading);
    }

    //same as above but with putting in target poset to compare with current instead of putting in heading
    public double HeadingErrorPose2d(Pose2d other) {
        double error = (other.heading - this.heading + Math.PI) % (2 * Math.PI) - Math.PI;
        return error;
    }

    //gives the angle you should be facing in radians, whereas heading error is how far off
    public double angleTo (Pose2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.atan2(dy, dx);
    }

    //gives pose between two poses, so the middle of the path
    //bit confused do later
    //do i even need this?
    /*
    public double middlePath(Pose2d other, double t) {

    }
    */

    private double normalizeAngle(double angle) {
        // On this edge case the servo will not move. If fixing the problem is less expensive than this, please do so.
        if (angle == 90) {
            angle = 89.999;
        }
        if (angle == -90 || angle == -180) {
            angle += 0.001;
        }

        // Check if the angle is already in the desired range.
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [-360, 360).
        double normalizedAngle = angle % 360;

        // If the result was negative, shift it to the range [0, 360).
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }

        // If the angle is in the range [180, 360), shift it to [-180, 0).
        if (normalizedAngle >= 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }
}