package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class LimelightLogic {
    private RobotContainer robot;
    private Telemetry telemetry;
    public Limelight3A limelight;
    private LLResult result;
    private ElapsedTime turretTime = new ElapsedTime();
    private Pose2D botPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private double p = 177.5;
    // 50, 100, 160
    private double[] table = {0.363, 0.43, 0.6};
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

    public void update() {
        if (limelight.getLatestResult().isValid()) {
            result = limelight.getLatestResult();
            if (Status.motif == null) findMotif();
            if (Status.alliance == null) findAlliance();
            if (Status.motif != null) {
                result.getFiducialResults().removeIf(tag -> tag.getFiducialId() == 21 || tag.getFiducialId() == 22 || tag.getFiducialId() == 23);
//                RobotContainer.HardwareDevices.pinpoint.setPosition(HelperFunctions.to2D(result.getBotpose()));
            }
        }
    }

    public void trackGoal() {
        if (limelight.getLatestResult() != null && Math.abs(limelight.getLatestResult().getTx()) > 1) {
            p += limelight.getLatestResult().getTx() * Constants.Turret.TRACK_GOAL_P;
            robot.turret.setTargetAngle(p);
            telemetry.addData("TX", limelight.getLatestResult().getTx());
            telemetry.addData("p", p);
        }
    }

    public Pose2D logicBotPose() {
        if (result != null) {
            double a = Turret.turretServoMaster.getPosition() - 0.5;
            double r = 6.819323 / 2.54; //68.19323mm
            botPose = new Pose2D(DistanceUnit.INCH, -((result.getBotpose().getPosition().x * 100) / 2.54) + (Math.cos(a) * r), (((result.getBotpose().getPosition().y * 100) / 2.54) + (Math.sin(a)) * r), AngleUnit.DEGREES, 0);
        }
        return botPose;
    }

    public double disToGoal() {
        if (result != null) {
            Pose2D botPose = logicBotPose();
            double xDiff = Constants.Game.GOAL_POSE.getX(DistanceUnit.INCH) - botPose.getX(DistanceUnit.INCH);
            double yDiff = Constants.Game.GOAL_POSE.getY(DistanceUnit.INCH) - botPose.getY(DistanceUnit.INCH);
            return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        } else {
            return 0;
        }
    }

    public double disToGoalPinpoint() {
            double xDiff = Constants.Game.GOAL_POSE.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
            double yDiff = Constants.Game.GOAL_POSE.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    public Constants.Game.MOTIF checkMotif(LLResultTypes.FiducialResult aprilTag) {
        if (aprilTag.getFiducialId() == 21) {
            return Constants.Game.MOTIF.GPP;
        } else if (aprilTag.getFiducialId() == 22) {
            return Constants.Game.MOTIF.PGP;
        } else if (aprilTag.getFiducialId() == 23) {
            return Constants.Game.MOTIF.PPG;
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

    public Constants.Game.ALLIANCE checkAlliance(LLResultTypes.FiducialResult aprilTag) {
        if (aprilTag.getFiducialId() == 20) {
            return Constants.Game.ALLIANCE.BLUE;
        } else if (aprilTag.getFiducialId() == 24) {
            return Constants.Game.ALLIANCE.RED;
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
    // Assuming point 1 is less than point 2
    public double interpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }

    public double useInterpolate() {
        // variable
        double dist = disToGoal();
        double turretSpeed = 0;
        if (dist == 0) {
            return 0;
        } else {
            if (dist < 100) {
                turretSpeed = interpolate(table[0], table[1], (dist - 50) / (100 - 50));
            } else {
                turretSpeed = interpolate(table[1], table[2], (dist - 100) / (160 - 100));
            }
        }
        return turretSpeed;
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
