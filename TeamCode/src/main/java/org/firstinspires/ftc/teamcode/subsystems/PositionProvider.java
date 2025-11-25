package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


public class PositionProvider {
    private RobotContainer robotContainer;
    private LimelightLogic limelightLogic;
    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime timer = new ElapsedTime();
    private Pose2D visionOffsetPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    public PositionProvider(RobotContainer robotContainer, LimelightLogic limelightLogic, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.limelightLogic = limelightLogic;
        this.pinpoint = pinpoint;
    }

    public Pose2D getRobotPose() {
        Pose2D odPose = pinpoint.getPosition();
        double rx = odPose.getX(DistanceUnit.CM) + visionOffsetPose.getX(DistanceUnit.CM);
        double ry = odPose.getY(DistanceUnit.CM) + visionOffsetPose.getY(DistanceUnit.CM);
        double rHeading = odPose.getHeading(AngleUnit.DEGREES) + visionOffsetPose.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.CM, rx, ry, AngleUnit.DEGREES, rHeading);
    }

    public void update() {
        Pose2D odPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
        Pose2D visionPose = getGoodLimeLightPose();
        if (visionPose != null) {
            timer.startTime();
            ArrayList<Pose2D> visionOffsetPoseList = new ArrayList<>();

            double llX = visionPose.getX(DistanceUnit.CM);
            double llY = visionPose.getY(DistanceUnit.CM);
            double llHeading = visionPose.getHeading(AngleUnit.DEGREES);

            visionOffsetPose = new Pose2D(DistanceUnit.CM, llX - odPose.getX(DistanceUnit.CM), llY - odPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, llHeading - odPose.getHeading(AngleUnit.DEGREES));
        }
        Status.currentPose = getRobotPose();
    }

    private Pose2D getGoodLimeLightPose() {
        LimeLightInfo info = limelightLogic.limelightInfo();
        if (info == null) {
            return null;
        }
        // If LimeLight result outside of the field ignore it
        double x = info.pose.getX(DistanceUnit.CM);
        double y = info.pose.getY(DistanceUnit.CM);
        double yaw = info.pose.getHeading(AngleUnit.DEGREES);
        // If outside the field x
        if (x < -1.8288 || x > 1.8288) {
            return null;
        }
        // If outside the field y
        if (y < -1.8288 || y > 1.8288) {
            return null;
        }
        // If yaw is impossible
        if (yaw > 360 || yaw < 0) {
            return null;
        }
        // If the april tag is to far from the center x
        if (info.result.getTx() < -20 || info.result.getTx() > 20) {
            return null;
        }
        // If the april tag is to far from the center y
        if (info.result.getTy() < -20 || info.result.getTy() > 20) {
            return null;
        }
        // Maybe more?
        return info.pose;
    }
}
