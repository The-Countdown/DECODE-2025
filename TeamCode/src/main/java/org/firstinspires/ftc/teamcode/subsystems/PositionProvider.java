package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.AndroidSerialNumberNotFoundException;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


public class PositionProvider {
    private RobotContainer robotContainer;
    private LimelightLogic limelightLogic;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D visionOffsetPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    private ElapsedTime visionTimer = new ElapsedTime();
    private ArrayList<Pose2D> visionPoseList = new ArrayList<>();
    private Pose2D lastODPose;

    public PositionProvider(RobotContainer robotContainer, LimelightLogic limelightLogic, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.limelightLogic = limelightLogic;
        this.pinpoint = pinpoint;
    }

    public Pose2D getRobotPose() {
        Pose2D odPose = pinpoint.getPosition();
        double rx = odPose.getX(DistanceUnit.CM) + visionOffsetPose.getX(DistanceUnit.CM);
        double ry = odPose.getY(DistanceUnit.CM) + visionOffsetPose.getY(DistanceUnit.CM);
        double rHeading = odPose.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.CM, rx, ry, AngleUnit.DEGREES, rHeading);
    }

    public Pose2D getVisionOffsetPose() {
        return visionOffsetPose;
    }

    @Deprecated
    public ArrayList<Pose2D> getVisionPoseList() {
        return visionPoseList;
    }

    public void update() {
        Pose2D odPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
        Pose2D visionPose = getGoodLimeLightPose();
        if (lastODPose == null) {
            lastODPose = odPose;
        }

        if (visionPose != null) {
            // If robot has not moved much
            if (Math.abs(lastODPose.getX(DistanceUnit.CM) - odPose.getX(DistanceUnit.CM)) < 0.1 && Math.abs(lastODPose.getY(DistanceUnit.CM) - odPose.getY(DistanceUnit.CM)) < 0.1) {
                visionPoseList.add(visionPose);
            } else {
                visionPoseList = new ArrayList<>();
            }
        } else {
            visionTimer.reset();
            visionPoseList = new ArrayList<>();
        }
        if (visionTimer.seconds() > 5 && visionPoseList.size() > 100)  {
            // Average all vision estimates over the time.
            double llX = 0;
            double llY = 0;
            for (int i = 0; i < visionPoseList.size(); i++) {
                llX += visionPoseList.get(i).getX(DistanceUnit.CM);
                llY += visionPoseList.get(i).getY(DistanceUnit.CM);
            }

            llX = llX / visionPoseList.size();
            llY = llY / visionPoseList.size();

            visionOffsetPose = new Pose2D(DistanceUnit.CM, llX - odPose.getX(DistanceUnit.CM), llY - odPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, 0);
            visionTimer.reset();
            visionPoseList = new ArrayList<>();
        }
        Status.currentPose = getRobotPose();
        lastODPose = odPose;
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
        if (Math.abs(x) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " x+");
            return null;
        }
        // If outside the field y
        if (Math.abs(y) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " y+");
            return null;
        }
        // If yaw is impossible
        if (yaw > 360 || yaw < 0) {
            robotContainer.telemetry.addData("Limelight Failed because", " yaw");
            return null;
        }
        // If the april tag is to far from the center x
        if (Math.abs(info.result.getTx()) > 8) {
            robotContainer.telemetry.addData("Limelight Failed because", " tx");
            return null;
        }
        // If the april tag is to far from the center y
        if (Math.abs(info.result.getTy()) > 12) {
            robotContainer.telemetry.addData("Limelight Failed because", " ty");
            return null;
        }
        // Maybe more?
        return info.pose;
    }
}
