package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.AndroidSerialNumberNotFoundException;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
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
        // Rotate the odPose by the vision offset
        Pose2D odPose = pinpoint.getPosition();
        double rx = (odPose.getX(DistanceUnit.CM) + visionOffsetPose.getX(DistanceUnit.CM)) * 0.9914129;
        double ry = (odPose.getY(DistanceUnit.CM) + visionOffsetPose.getY(DistanceUnit.CM)) * 0.998610856289;
        double rh = visionOffsetPose.getHeading(AngleUnit.RADIANS);
        double newX = rx * Math.cos(-rh) - ry * Math.sin(-rh);
        double newY = ry * Math.cos(-rh) + rx * Math.sin(-rh);
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
            if (robotContainer.gamepadEx1.dpadDown.isHeld()) {
                LimeLightInfo visionInfo = getGoodLimeLightInfo();

                if (visionInfo != null) {
                    double lateralDistance = visionInfo.result.getTx();
                    Status.correctionDegrees += lateralDistance * HelperFunctions.disToGoal() * Constants.Robot.CORRECTION_DEGREES_MULTIPLIER;

                    double newX = odPose.getX(DistanceUnit.CM) * Math.cos(-Math.toRadians(Status.correctionDegrees)) - odPose.getY(DistanceUnit.CM) * Math.sin(-Math.toRadians(Status.correctionDegrees));
                    double newY = odPose.getY(DistanceUnit.CM) * Math.cos(-Math.toRadians(Status.correctionDegrees)) + odPose.getX(DistanceUnit.CM) * Math.sin(-Math.toRadians(Status.correctionDegrees));

                    visionOffsetPose = new Pose2D(DistanceUnit.CM, newX - odPose.getX(DistanceUnit.CM), newY - odPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, Status.correctionDegrees);
                }
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
    private LimeLightInfo getGoodLimeLightInfo() {
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
        return info;
    }
}
