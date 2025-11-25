package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


public class positionProvider {
    private RobotContainer robotContainer;
    private LimelightLogic limelightLogic;
    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime timer = new ElapsedTime();
    private Pose2D visionOffsetPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    public positionProvider(RobotContainer robotContainer, LimelightLogic limelightLogic, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.limelightLogic = limelightLogic;
        this.pinpoint = pinpoint;
    }

    public void update() {
        Pose2D odPose = Status.currentPose;
        if (getGoodLimeLightPose() != null) {
            timer.startTime();
            ArrayList<Pose2D> visionOffsetPoseList = new ArrayList<>();

            double llX = limelightLogic.limelight.getLatestResult().getBotpose().getPosition().x;
            double llY = limelightLogic.limelight.getLatestResult().getBotpose().getPosition().y;
            double llHeading = limelightLogic.limelight.getLatestResult().getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);

            visionOffsetPoseList.add(new Pose2D(DistanceUnit.CM, llX - odPose.getX(DistanceUnit.CM), llY - odPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, llHeading - odPose.getHeading(AngleUnit.DEGREES)));

            if (timer.milliseconds() >= Constants.Pathing.LIMELIGHT_UPDATE_AVERAGING_MS) {
               double xDiff = 0;
               double yDiff = 0;
               double headingDiff = 0;
                for (int i = 0; i < visionOffsetPoseList.size(); i++) {
                    xDiff += visionOffsetPoseList.get(i).getX(DistanceUnit.CM);
                    yDiff += visionOffsetPoseList.get(i).getY(DistanceUnit.CM);;
                    headingDiff += visionOffsetPoseList.get(i).getHeading(AngleUnit.DEGREES);
                }
                xDiff = (xDiff/visionOffsetPoseList.size()) - odPose.getX(DistanceUnit.CM);
                yDiff = (yDiff/visionOffsetPoseList.size()) - odPose.getY(DistanceUnit.CM);
                headingDiff = (headingDiff/visionOffsetPoseList.size()) - odPose.getHeading(AngleUnit.DEGREES);
                visionOffsetPose = new Pose2D(DistanceUnit.CM, xDiff, yDiff, AngleUnit.DEGREES, headingDiff);
                timer.reset();
            }

        }
    }

    private LLResult getGoodLimeLightPose() {
        LLResult result = limelightLogic.limelight.getLatestResult();
        if (result == null) {
            return null;
        }
        if (Math.abs(limelightLogic.limelight.getLatestResult().getBotpose().getPosition().x) < 1.8288 && Math.abs(limelightLogic.limelight.getLatestResult().getBotpose().getPosition().y) < 1.8288) {
            return result;
        }
        return null;
    }
}
