package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class DelayedActionManager {
    private final List<DelayedAction> delayedActions = new ArrayList<>();
    List<Double> timeAtPause = new ArrayList<>();
    boolean paused = false;

    public void schedule(DelayedAction delayedAction) {
        delayedActions.add(delayedAction);
    }
    public void schedule(Runnable action, int delayMs) {
        delayedActions.add(new DelayedAction(action, delayMs));
    }
    public void schedule(Runnable action, BooleanSupplier condition) {
        delayedActions.add(new DelayedAction(action, condition));
    }
    public void schedule(Runnable action, BooleanSupplier condition, int timeoutMs) {
        delayedActions.add(new DelayedAction(action, condition, timeoutMs));
    }

    public void update() {
        if (!paused) {
            for (DelayedAction delayedAction : delayedActions) {
                if (delayedAction.shouldExecute()) {
                    delayedAction.execute();
                }
            }
            delayedActions.removeIf(DelayedAction::isExecuted);
        }
    }

    public void scheduleSequence(List<DelayedAction> actions) {
        int cumulativeDelay = 0;
        for (DelayedAction delayedAction : actions) {
            cumulativeDelay += delayedAction.delayMs;
            delayedActions.add(new DelayedAction(delayedAction.action, cumulativeDelay));
        }
    }

    public void cancelAll() {
        for (DelayedAction delayedAction : delayedActions) {
            delayedAction.cancel();
        }
        delayedActions.clear();
    }

    public void pause() {
        paused = true;
        for (DelayedAction delayedAction : delayedActions) {
            delayedAction.pause();
        }
    }

    public void resume() {
        paused = false;
        for (DelayedAction delayedAction : delayedActions) {
            delayedAction.resume();
        }
    }

    public static class DelayedAction {
        public Runnable action;
        public int delayMs;
        public BooleanSupplier condition;
        public ElapsedTime timer = new ElapsedTime();
        private boolean executed = false;
        private boolean cancelled = false;
        private boolean paused = false;

        private double pauseTime = 0;
        private double accumulatedPausedTime = 0;

        public DelayedAction(Runnable action, int delayMs) {
            this.action = action;
            this.delayMs = delayMs;
        }

        public DelayedAction(Runnable action, BooleanSupplier condition) {
            this.action = action;
            this.condition = condition;
            this.delayMs = -1;
        }

        public DelayedAction(Runnable action, BooleanSupplier condition, int timeoutMs) {
            this.action = action;
            this.condition = condition;
            this.delayMs = timeoutMs;
        }

        public boolean shouldExecute() {
            if (executed || cancelled || paused) return false;

            double currentTime = timer.milliseconds() - accumulatedPausedTime;

            if (condition != null) {
                return condition.getAsBoolean() || (timer != null && currentTime >= delayMs);
            } else {
                return timer != null && currentTime >= delayMs;
            }
        }

        public void execute() {
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
            if (!paused) {
                pauseTime = timer.milliseconds();
                paused = true;
            }
        }

        public void resume() {
            if (paused) {
                accumulatedPausedTime += (timer.milliseconds() - pauseTime);
                paused = false;
            }
        }
    }
}
