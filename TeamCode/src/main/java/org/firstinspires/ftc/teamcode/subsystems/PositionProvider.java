package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.AndroidSerialNumberNotFoundException;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;

import java.util.ArrayList;


public class PositionProvider {
    private RobotContainer robotContainer;
    private LimelightLogic limelightLogic;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D visionOffsetPose;
    private ElapsedTime visionTimer = new ElapsedTime();
    private ArrayList<Pose2D> visionPoseList;
    private Pose2D lastODPose;
    private Pose2D startODPose;

    public PositionProvider(RobotContainer robotContainer, LimelightLogic limelightLogic, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.limelightLogic = limelightLogic;
        this.pinpoint = pinpoint;
        this.visionOffsetPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        this.startODPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        this.visionPoseList = new ArrayList<>();
    }

    public Pose2D getRobotPose() {
        Pose2D odPose = pinpoint.getPosition();
        double rx = (odPose.getX(DistanceUnit.CM) + visionOffsetPose.getX(DistanceUnit.CM)) * 0.9914129;
        double ry = (odPose.getY(DistanceUnit.CM) + visionOffsetPose.getY(DistanceUnit.CM)) * 0.998610856289;
        double rHeading = (odPose.getHeading(AngleUnit.DEGREES) + visionOffsetPose.getHeading(AngleUnit.DEGREES));
        return new Pose2D(DistanceUnit.CM, rx, ry, AngleUnit.DEGREES, rHeading);
    }

    public Pose2D getVisionOffsetPose() {
        return visionOffsetPose;
    }

    @Deprecated
    public ArrayList<Pose2D> getVisionPoseList() {
        return visionPoseList;
    }

    public void update(boolean LimeLight) {
        if (LimeLight) {
            Pose2D odPose = RobotContainer.HardwareDevices.pinpoint.getPosition();

            Pose2D visionPose = getGoodLimeLightPose();
            if (lastODPose == null) {
                lastODPose = odPose;
            }

            if (visionPose != null) {
                if (startODPose == null) {
                    startODPose = odPose;
                }
                // If robot has not moved much
                if (Math.abs(startODPose.getX(DistanceUnit.CM) - odPose.getX(DistanceUnit.CM)) < 2 && Math.abs(startODPose.getY(DistanceUnit.CM) - odPose.getY(DistanceUnit.CM)) < 2 && Math.abs(startODPose.getHeading(AngleUnit.DEGREES) - odPose.getHeading(AngleUnit.DEGREES)) < 4 && robotContainer.gamepadEx1.dpadDown.isHeld()) {
//                if (robotContainer.gamepadEx1.rightStickX() == 0 && robotContainer.gamepadEx1.leftStickX() == 0 && robotContainer.gamepadEx1.leftStickY() == 0) {
                    visionPoseList.add(visionPose);
                } else {
                    visionTimer.reset();
                    visionPoseList = new ArrayList<>();
                    startODPose = odPose;
                }
            } else {
                visionTimer.reset();
                visionPoseList = new ArrayList<>();
                startODPose = odPose;
            }
            // Add something about rejecting poses that are far from the current average
            if (visionTimer.seconds() > 1 && visionPoseList.size() > 80) {
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
                startODPose = odPose;
            }
            Status.currentPose = getRobotPose();
            lastODPose = odPose;
        } else {
           Status.currentPose = new Pose2D(DistanceUnit.CM, RobotContainer.HardwareDevices.pinpoint.getPosX(DistanceUnit.CM), RobotContainer.HardwareDevices.pinpoint.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, RobotContainer.HardwareDevices.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        }
    }

    private Pose2D getGoodLimeLightPose() {
        LimeLightInfo info = limelightLogic.logicBotPoseCM();
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
//        if (yaw > 360 || yaw < 0) {
//            robotContainer.telemetry.addData("Limelight Failed because", " yaw");
//            return null;
//        }
        robotContainer.telemetry.addData("llyaw", yaw);
        // If the april tag is to far from the center x
        if (Math.abs(info.result.getTx()) > 12) {
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
