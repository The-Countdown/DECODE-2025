package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class LimelightLogic {
    RobotContainer robot;
    Telemetry telemetry;
    Limelight3A limelight;
    LLResult result;

    public LimelightLogic(RobotContainer robot, Telemetry telemetry, Limelight3A limelight) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.limelight = limelight;
    }

    public void updateLimelight() {
        if (limelight.getLatestResult().isValid()) result = limelight.getLatestResult();
        if (Status.motif == null) findMotif();
        if (Status.alliance == null) findAlliance();
        RobotContainer.HardwareDevices.pinpoint.setPosition(HelperFunctions.to2D(result.getBotpose()));
    }

    public Constants.MOTIF checkMotif(LLResultTypes.FiducialResult aprilTag) {
        if (aprilTag.getFiducialId() == 21) {
            return Constants.MOTIF.GPP;
        } else if (aprilTag.getFiducialId() == 22) {
            return Constants.MOTIF.PGP;
        } else if (aprilTag.getFiducialId() == 23) {
            return Constants.MOTIF.PPG;
        }
        return null;
    }

    public void findMotif() {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (checkMotif(tag) != null) {
                Status.motif = checkMotif(tag);
                robot.addRetainedTelemetry("Motif Found" , checkMotif(tag));
            }
        }
    }

    public Constants.ALLIANCE checkAlliance(LLResultTypes.FiducialResult aprilTag) {
        if (aprilTag.getFiducialId() == 20) {
            return Constants.ALLIANCE.BLUE;
        } else if (aprilTag.getFiducialId() == 24) {
            return Constants.ALLIANCE.RED;
        } else
            return null;
    }

    public void findAlliance() {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (checkMotif(tag) != null) {
                Status.alliance = checkAlliance(tag);
                robot.addRetainedTelemetry("Motif Found" , checkMotif(tag));
            }
        }
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
