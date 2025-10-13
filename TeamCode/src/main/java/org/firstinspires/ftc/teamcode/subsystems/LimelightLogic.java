package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LinkedServos;

public class LimelightLogic {
    private RobotContainer robot;
    private Telemetry telemetry;
    private Limelight3A limelight;
    private LLResult result;
    private ElapsedTime turretTime = new ElapsedTime();
    private double p = 177.5;
    public LimelightLogic(RobotContainer robot, Telemetry telemetry, Limelight3A limelight) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.limelight = limelight;


        LLFieldMap field = new LLFieldMap();
//        List<Double> transform = List.of(1.1, 2.1, 3.1);
//
//        new LLFieldMap.Fiducial(20, 6, "fam", transform, true);
//
//
//        limelight.uploadFieldmap(field, null);
    }

    public void updateLimelight() {
        if (limelight.getLatestResult().isValid()) {
            result = limelight.getLatestResult();
            if (Status.motif == null) findMotif();
            if (Status.alliance == null) findAlliance();
            if (Status.motif != null) {
                result.getFiducialResults().removeIf(tag -> tag.getFiducialId() == 21 || tag.getFiducialId() == 22 || tag.getFiducialId() == 23);
                RobotContainer.HardwareDevices.pinpoint.setPosition(HelperFunctions.to2D(result.getBotpose()));
            }
        }
    }

    public void trackGoal() {
        if (limelight.getLatestResult().isValid() && Math.abs(limelight.getLatestResult().getTx()) > 1) {
            p += limelight.getLatestResult().getTx() * Constants.TRACK_GOAL_P;
            robot.turret.setTurretTargetAngle(p);
            telemetry.addData("TX", limelight.getLatestResult().getTx());
            telemetry.addData("p", p);
        } else {
            robot.turret.setTurretTargetAngle(177.5);
        }
    }

    public Pose3D getPose() {
        if (limelight.getLatestResult().isValid()) {
            return result.getBotpose();
        } else {
            return new Pose3D(new Position(DistanceUnit.CM, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        }
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
                robot.addRetainedTelemetry("Alliance Found" , checkAlliance(tag));
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
