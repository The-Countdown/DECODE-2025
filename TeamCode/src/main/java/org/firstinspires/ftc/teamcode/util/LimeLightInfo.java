package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LimeLightInfo {
    public Pose2D pose ;
    public LLResult result;
    public LimeLightInfo(Pose2D pose, LLResult result) {
        this.pose = pose;
        this.result = result;
    }
}
