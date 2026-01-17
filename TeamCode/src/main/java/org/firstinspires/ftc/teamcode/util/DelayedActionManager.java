package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class DelayedActionManager {
    RobotContainer robotContainer;
    private final List<Action> delayedActions = new ArrayList<>();
    volatile boolean enabled;
    public static int currentPose;
    private long updateCycle = 0;

    public DelayedActionManager(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.currentPose = 0;
        this.enabled = true;
    }

    public void schedule(DelayedAction delayedAction) {
        delayedActions.add(delayedAction);
    }
    public void schedule(Runnable action, int delayMs) {
        delayedActions.add(new DelayedAction(robotContainer, action, delayMs));
    }
    public void schedule(Runnable[] action, int delayMs) {
        for (Runnable runnable : action) {
            delayedActions.add(new DelayedAction(robotContainer, runnable, delayMs));
        }
    }

    public void schedule(Runnable action, BooleanSupplier condition) {
        delayedActions.add(new DelayedAction(robotContainer, action, condition));
    }

    public void schedule(Runnable action, BooleanSupplier condition, boolean waitForPreviousAction) {
        delayedActions.add(new DelayedAction(robotContainer, action, condition, waitForPreviousAction, delayedActions.isEmpty() ? null : delayedActions.get(delayedActions.size() - 1)));
    }

    public void schedule(Runnable action, BooleanSupplier condition, int delay, boolean waitForPreviousAction) {
        delayedActions.add(new DelayedAction(robotContainer, action, condition, delay, waitForPreviousAction, delayedActions.isEmpty() ? null : delayedActions.get(delayedActions.size() - 1)));
    }

    public void schedule(Runnable action, BooleanSupplier condition, int delayMs) {
        delayedActions.add(new DelayedAction(robotContainer, action, condition, delayMs));
    }

    public void schedulePose(Runnable action) {
        delayedActions.add(new PoseAction(robotContainer, action, currentPose));
    }

    public void schedulePose(Runnable action, int poseDelay) {
        delayedActions.add(new PoseAction(robotContainer, action, poseDelay));
    }

    public void schedulePose(Runnable action, BooleanSupplier condition) {
        delayedActions.add(new PoseAction(robotContainer, action, condition, currentPose));
    }
    
    public void schedulePose(Runnable action, BooleanSupplier condition, int poseDelay) {
        delayedActions.add(new PoseAction(robotContainer, action, condition, poseDelay));
    }

    public void incrementPoseOffset() {
        currentPose += 1;
    }

    public void incrementPoseOffset(int amount) {
        currentPose += amount;
    }

    public void setPoseOffset(int index) {
        currentPose = index;
    }

    public synchronized void update() {
        updateCycle++;
        if (enabled) {
            for (int i = 0; i < delayedActions.toArray().length; i++) {
                Action delayedAction = delayedActions.get(i);
                if (delayedAction.shouldExecute()) {
                    delayedAction.execute();
                }
                robotContainer.telemetry.addData(i + "shouldExecute", delayedAction.shouldExecute());
                robotContainer.telemetry.addData(i + "executed", delayedAction.executed);
            }
        }
    }

    public void scheduleSequence(List<DelayedAction> actions) {
        int cumulativeDelay = 0;
        for (DelayedAction delayedAction : actions) {
            cumulativeDelay += delayedAction.delayMs;
            delayedActions.add(new DelayedAction(robotContainer, delayedAction.action, cumulativeDelay));
        }
    }

    public void cancelAll() {
        for (Action delayedAction : delayedActions) {
            delayedAction.cancel();
        }
        currentPose = 0;
        delayedActions.clear();
    }

    public void pause() {
        enabled = false;
        for (Action delayedAction : delayedActions) {
            delayedAction.pause();
        }
    }

    public void resumeAll() {
        enabled = true;
        for (Action delayedAction : delayedActions) {
            delayedAction.resume();
        }
    }

    public static class Action {
        public RobotContainer robotContainer;
        public Runnable action;
        public BooleanSupplier condition;
        public ElapsedTime timer = new ElapsedTime();
        public boolean executed = false;
        public boolean cancelled = false;
        public boolean enabled = true;
        public boolean waitForPreviousAction = false;
        public long executedAtLoop = -1;
        public Action previousAction = null;

        public double pauseTime = 0;
        public double accumulatedPausedTime = 0;
        public boolean shouldExecute() {
            return false;
        }

        public synchronized void execute() {
            if (!executed && !cancelled) {
                action.run();
                executed = true;
                executedAtLoop = robotContainer.delayedActionManager.updateCycle;
            }
        }

        public void forceExecute() {
            action.run();
        }

        public boolean isExecuted() {
            return executed || cancelled;
        }

        public void cancel() {
            cancelled = true;
        }

        public void pause() {
            if (enabled) {
                pauseTime = timer.milliseconds();
                enabled = false;
            }
        }

        public void resume() {
            if (!enabled) {
                accumulatedPausedTime += (timer.milliseconds() - pauseTime);
                enabled = true;
            }
        }
    }

    public static class DelayedAction extends Action {
        public int delayMs;

        public DelayedAction(RobotContainer robotContainer, Runnable action, int delayMs) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.delayMs = delayMs;
        }

        public DelayedAction(RobotContainer robotContainer, Runnable action, BooleanSupplier condition) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.condition = condition;
            this.delayMs = -1;
        }

        public DelayedAction(RobotContainer robotContainer, Runnable action, BooleanSupplier condition, boolean waitForPreviousAction, Action previousAction) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.condition = condition;
            this.delayMs = -1;
            this.waitForPreviousAction = waitForPreviousAction;
            this.previousAction = previousAction;
        }

        public DelayedAction(RobotContainer robotContainer, Runnable action, BooleanSupplier condition, int delayMs, boolean waitForPreviousAction, Action previousAction) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.condition = condition;
            this.delayMs = delayMs;
            this.waitForPreviousAction = waitForPreviousAction;
            this.previousAction = previousAction;
        }

        public DelayedAction(RobotContainer robotContainer, Runnable action, BooleanSupplier condition, int delayMs) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.condition = condition;
            this.delayMs = delayMs;
        }

        @Override
        public boolean shouldExecute() {
            if (executed || cancelled || !enabled) return false;
            if (waitForPreviousAction && previousAction != null) {
                if (!previousAction.isExecuted()) {
                    timer.reset();
                    return false;
                }
                if (previousAction.executedAtLoop == robotContainer.delayedActionManager.updateCycle) {
                    timer.reset();
                    return false;
                }
            }

            double currentTime = timer.milliseconds() - accumulatedPausedTime;

            boolean delaySatisfied = delayMs < 1 || (timer != null && currentTime >= delayMs);

            if (condition != null) {
                return delaySatisfied || condition.getAsBoolean();
            } else {
                return delaySatisfied;
            }
        }
    }
    public static class PoseAction extends Action {
        public int poseDelay;

        public PoseAction(RobotContainer robotContainer, Runnable action, int poseDelay) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.poseDelay = poseDelay;
        }
        public PoseAction(RobotContainer robotContainer, Runnable action, BooleanSupplier condition, int poseDelay) {
            this.robotContainer = robotContainer;
            this.action = action;
            this.condition = condition;
            this.poseDelay = poseDelay;
        }

        public boolean shouldExecute() {
            if (executed || cancelled || !enabled) return false;

            double currentTime = timer.milliseconds() - accumulatedPausedTime;

            if (condition != null) {
                return condition.getAsBoolean() || (timer != null && robotContainer.pathPlanner.currentPath == poseDelay);
            } else {
                return timer != null && robotContainer.pathPlanner.currentPath >= poseDelay;
            }
        }

        public synchronized void execute() {
            if (!executed && !cancelled) {
                action.run();
                executed = true;
            }
        }

        public boolean isExecuted() {
            return executed || cancelled;
        }

        public void cancel() {
            cancelled = true;
        }

        public void pause() {
            if (enabled) {
                pauseTime = timer.milliseconds();
                enabled = false;
            }
        }

        public void resume() {
            if (!enabled) {
                accumulatedPausedTime += (timer.milliseconds() - pauseTime);
                enabled = true;
            }
        }
    }
}
