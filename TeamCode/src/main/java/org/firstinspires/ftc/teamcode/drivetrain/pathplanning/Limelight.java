package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import static org.firstinspires.ftc.teamcode.main.RobotContainer.HardwareDevices.limelight;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Limelight {
    RobotContainer robot;
    Telemetry telemetry;
    Limelight limelightCam;

    public Limelight(RobotContainer robot, Telemetry telemetry, Limelight limelight) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.limelightCam = limelight;
    }

    LLResult result =  limelight.getLatestResult();

    /**
    * @param pose Pose3d to convert to Pose2d
    * @return Pose2d
    */
    public Pose2D to2D(Pose3D pose) {
        return new Pose2D(DistanceUnit.CM, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES));
    }


    public int getTag() {
        return limelightCam.getTag();
    }

    public Pose3D getRobotPose() {
        if (result.isValid()) {
            return result.getBotpose();
        } else {
            return null;
        }
    }

    public void updateLimelight() {
        RobotContainer.HardwareDevices.pinpoint.setPosition(limelightCam.to2D(getRobotPose()));
    }

    public enum motif {
        GPP,
        PGP,
        PPG
    }

    public motif getMotif(LLFieldMap.Fiducial aprilTag) {

        if (aprilTag.getId() == 21) {
            return motif.GPP;
            //GPP
        } else if (aprilTag.getId() == 22) {
            return motif.PGP;
            //PGP
        } else if (aprilTag.getId() == 23) {
            return motif.PPG;
            //PPG
        }
        return null;
    }

    public char getAlliance(LLFieldMap.Fiducial aprilTag) {
        if (aprilTag.getId() == 20) {
            return 'B';
            //BLEu
        } else if (aprilTag.getId() == 24) {
            return 'R';
            //RED
        } return 'N';
    }


    // * look at april tag give pose2d (for initialization)
    // track april tag constantly (with offset) (with turret)
    // * look at random thing for color combo
    // drive to function with offset options
    // * get alliance

    //ID 21 GPP
    //ID 22 PGP
    //ID 23 PPG
    //ID 20 BLEU
    //ID 24 RED

}
