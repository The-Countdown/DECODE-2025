package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.LimeLightInfo;

public class LimelightLogic {
    private RobotContainer robotContainer;
    private Telemetry telemetry;
    public Limelight3A limelight;
    private LLResult result;
    private Pose2D botPose;
    private double p = 177.5;

    public LimelightLogic(RobotContainer robot, Telemetry telemetry, Limelight3A limelight) {
        this.robotContainer = robot;
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.botPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        LLFieldMap field = new LLFieldMap();
    }

    public void update() {
        if (limelight.getLatestResult().isValid()) {
            result = limelight.getLatestResult();
//             if (Status.motif == null) findMotif();
//             if (Status.alliance == null) findAlliance();
//             if (Status.motif != null) {
//                 result.getFiducialResults().removeIf(tag -> tag.getFiducialId() == 21 || tag.getFiducialId() == 22 || tag.getFiducialId() == 23);

                // Limelight position Updater
//                 RobotContainer.HardwareDevices.pinpoint.setPosition(logicBotPoseCM().pose);

                // Turret tracking fallback
//                 pointAtGoal();
//
        }
    }

    public boolean hasResult() {
        if (result == null) {
            return false;
        } else {
            return true;
        }
    }

    @Deprecated
    public void trackGoal() {
        if (limelight.getLatestResult() != null && Math.abs(limelight.getLatestResult().getTx()) > 1) {
            p += limelight.getLatestResult().getTx() * Constants.Turret.TRACK_GOAL_P;
            robotContainer.turret.setTargetAngle(p);
        }
    }

    @Deprecated
    public Pose2D limelightBotPose() {
        if (result != null) {
            double a = Turret.turretServoMaster.getPosition() - 0.5;
            double r = 6.819323 * 2.54; // 6.819323cm to inch
            botPose = new Pose2D(DistanceUnit.INCH, -((result.getBotpose().getPosition().x * 100) / 2.54) + (Math.cos(a) * r), (((result.getBotpose().getPosition().y * 100) / 2.54) + (Math.sin(a)) * r), AngleUnit.DEGREES, 0);
        }
        return botPose;
    }

//    public LimeLightInfo limelightInfo() {
//        if (result != null) {
//            double a = Turret.turretServoMaster.getPosition() - 0.5;
//            double r = 6.819323 / 2.54; // This should in fact be a negative I think
//            botPose = new Pose2D(DistanceUnit.INCH, -((result.getBotpose().getPosition().x * 100) / 2.54) + (Math.cos(a) * r), (((result.getBotpose().getPosition().y * 100) / 2.54) + (Math.sin(a)) * r), AngleUnit.DEGREES, 0);
//            return new LimeLightInfo(botPose, result);
//        }
//        return null;
//    }

    public LimeLightInfo logicBotPoseCM() {
        if (result == null) return null;

        final double r_cm = 7.026882;           // camera radial distance from turret center in cm (fix if wrong)
        final double TURRET_TRAVEL_DEGREES = 180.0; // total servo travel degrees
        final double SERVO_CENTER = 0.5;        // servo value that corresponds to "turret = 0°"

        // 1) map servo position -> turret angle in degrees (CW positive)
        double servoAngle = HelperFunctions.clamp(robotContainer.turret.getPositionDegrees(),-90, 125); // ONLY WORKS IF THIS ACCURATELY RETURNS -180 TO 180 WITH CENTER BEING 0 DEGREES

        double turretRad = Math.toRadians(servoAngle);

        // 3) camera position in robot-frame assuming at start (turret=0) camera is at (r,0)
        //    and when turret rotates, camera robot-frame pos = rotate((r,0), turretRad)
        double camX_robot = -r_cm * Math.cos(turretRad);
        double camY_robot = -r_cm * Math.sin(turretRad);

        // 4) camera displacement from starting pose (start at (r,0)):
        double dispX_robot = camX_robot + r_cm; // = r*(cos(theta)-1)
        double dispY_robot = camY_robot - 0.0;  // = r*sin(theta)

        // 5) get the pose reported by Limelight and convert to cm
        double reportedX_cm = result.getBotpose().getPosition().x * 100.0;
        double reportedY_cm = result.getBotpose().getPosition().y * 100.0;

        // 6) rotate camera displacement (robot-frame) into field-frame using robot heading
        //    (assumes result.getBotpose().getOrientation().getYaw() returns robot heading in degrees, CCW positive)
        double robotHeadingDeg = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES) + servoAngle;
        double headingRad = Math.toRadians(robotHeadingDeg);

        double dispX_field = dispX_robot * Math.cos(headingRad) - dispY_robot * Math.sin(headingRad);
        double dispY_field = dispX_robot * Math.sin(headingRad) + dispY_robot * Math.cos(headingRad);

        // 7) subtract the camera displacement (in field frame) from the reported (camera) pose
        //    to get the robot center pose in field frame
        double robotX_cm = reportedX_cm - dispX_field;
        double robotY_cm = reportedY_cm - dispY_field;

        botPose = new Pose2D(DistanceUnit.CM, -robotX_cm, -robotY_cm, AngleUnit.DEGREES, robotHeadingDeg);
        return new LimeLightInfo(botPose, result);
    }

    @Deprecated
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

    // @Deprecated
    // public void findMotif() {
    //     for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
    //         if (checkMotif(tag) != null) {
    //             Status.motif = checkMotif(tag);
    //             switch(Status.motif){
    //                 case PPG:
    //                     Status.ballsToShootOrder[0] = 0;
    //                     Status.ballsToShootOrder[1] = 0;
    //                     Status.ballsToShootOrder[2] = 1;
    //                     break;
    //                 case PGP:
    //                     Status.ballsToShootOrder[0] = 0;
    //                     Status.ballsToShootOrder[1] = 1;
    //                     Status.ballsToShootOrder[2] = 0;
    //                     break;
    //                 case GPP:
    //                     Status.ballsToShootOrder[0] = 1;
    //                     Status.ballsToShootOrder[1] = 0;
    //                     Status.ballsToShootOrder[2] = 0;
    //                     break;
    //                 default:
    //                     break;
    //             }
    //
    //             robotContainer.addRetainedTelemetry("Motif Found", checkMotif(tag));
    //         }
    //     }
    // }

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
                robotContainer.addEventTelemetry("Alliance Found", checkAlliance(tag));
            }
        }
    }

    public void limelightLocalization() {
        if (limelight.getLatestResult() != null) {
            RobotContainer.HardwareDevices.pinpoint.setPosY(-limelight.getLatestResult().getBotpose_MT2().getPosition().y * 100, DistanceUnit.CM);
            RobotContainer.HardwareDevices.pinpoint.setPosX(-limelight.getLatestResult().getBotpose_MT2().getPosition().x * 100, DistanceUnit.CM);
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

    public void pointAtGoal() {
        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }
        LLResult results = limelight.getLatestResult();

        if (results != null) {
            double tx = results.getTx();

            robotContainer.turret.setTargetAngle((tx * Constants.Turret.LIMELIGHT_TX_MULTIPLIER) + RobotContainer.HardwareDevices.betterIMU.getAngle());
        } else {
            robotContainer.turret.pointAtGoal();
        }
    }
}

