package org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;

import java.util.ArrayList;

public class ActionPose extends GeneralPose {
    private ArrayList<DelayedActionManager.Action> delayedActions;

    public ActionPose(RobotContainer robotContainer, Runnable... actions) {
        delayedActions = new ArrayList<>();
        for (int i = 0; i < actions.length; i++) {
            this.delayedActions.add(new DelayedActionManager.DelayedAction(robotContainer, actions[i], 0));
        }
    }

    @Override
    public boolean runActions() {
        for (int i = 0; i < this.delayedActions.size(); i++) {
            this.delayedActions.get(i).execute();
        }
        return true;
    }
}
