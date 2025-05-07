package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadWrapper {
    public static class ButtonReader {
        private boolean prevState = false;
        private boolean currState = false;
        private final ElapsedTime holdDuration = new ElapsedTime();
        private boolean isTiming = false;

        public void update(boolean newState) {
            prevState = currState;
            currState = newState;

            if(wasJustPressed()){
                holdDuration.reset();
                isTiming = true;
            } else if(wasJustReleased()){
                isTiming = false;
            }
        }

        public boolean wasJustPressed() {
            return currState && !prevState;
        }

        public boolean wasJustReleased() {
            return !currState && prevState;
        }

        public boolean isHeld() {
            return currState;
        }

        /**
         * Checks if the button has been held down for at least the specified number of seconds.
         * @param seconds The number of seconds to check for.
         * @return True if the button has been held for at least the specified number of seconds, false otherwise.
         */
        public boolean isHeldFor(double seconds) {
            if(isTiming){
                return holdDuration.seconds() >= seconds;
            }
            return false;
        }
    }

    private final Gamepad gamepad;

    public final ButtonReader a = new ButtonReader();
    public final ButtonReader b = new ButtonReader();
    public final ButtonReader x = new ButtonReader();
    public final ButtonReader y = new ButtonReader();

    public final ButtonReader dpadUp = new ButtonReader();
    public final ButtonReader dpadDown = new ButtonReader();
    public final ButtonReader dpadLeft = new ButtonReader();
    public final ButtonReader dpadRight = new ButtonReader();

    public final ButtonReader leftBumper = new ButtonReader();
    public final ButtonReader rightBumper = new ButtonReader();

    public final ButtonReader leftTrigger = new ButtonReader();
    public final ButtonReader rightTrigger = new ButtonReader();

    public final ButtonReader leftStickX = new ButtonReader();
    public final ButtonReader leftStickY = new ButtonReader();
    public final ButtonReader rightStickX = new ButtonReader();
    public final ButtonReader rightStickY = new ButtonReader();

    public GamepadWrapper(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);

        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);

        leftTrigger.update(gamepad.left_trigger > 0.1);
        rightTrigger.update(gamepad.right_trigger > 0.1);

        leftStickX.update(Math.abs(gamepad.left_stick_x) > 0);
        leftStickY.update(Math.abs(gamepad.left_stick_y) > 0);

        rightStickX.update(Math.abs(gamepad.right_stick_x) > 0);
        rightStickY.update(Math.abs(gamepad.right_stick_y) > 0);

    }

    public float leftStickX() {
        return gamepad.left_stick_x;
    }
    public float leftStickY() {
        return gamepad.left_stick_y;
    }
    public float rightStickX() {
        return gamepad.right_stick_x;
    }
    public float rightStickY() {
        return gamepad.right_stick_y;
    }

    public float leftTriggerRaw() {
        return gamepad.left_trigger;
    }
    public float rightTriggerRaw() {
        return gamepad.right_trigger;
    }
}
