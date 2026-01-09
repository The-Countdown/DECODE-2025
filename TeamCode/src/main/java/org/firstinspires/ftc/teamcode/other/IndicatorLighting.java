package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class IndicatorLighting {
    public static class Group {
        private final List<Light> lights = new ArrayList<>();
        private final RobotContainer robotContainer;
        private final ElapsedTime lightTimer = new ElapsedTime();

        public Group(RobotContainer robotContainer) {
            this.robotContainer = robotContainer;
        }

        public void addLight(RobotContainer container, ServoImplEx hardware) {
            lights.add(new Light(container, hardware));
        }

        public void addLight(Light light) {
            lights.add(light);
        }

        public void setColor(Constants.LED.COLOR color) {
            for (Light light : lights) {
                light.setColor(color);
            }
            Status.currentLightColor = color;
        }

        public void rainbow() {
            for (Light light : lights) {
                light.rainbow();
            }
        }

        public void police() {
            lights.get(0).flashing(Constants.LED.COLOR.RED, Constants.LED.COLOR.BLUE, 1);
            lights.get(1).flashing(Constants.LED.COLOR.BLUE, Constants.LED.COLOR.RED, 1);
        }

        public void rainbowReset() {
            for (Light light : lights) {
                light.rainbowReset();
            }
        }

        public void flashing(Constants.LED.COLOR color1, Constants.LED.COLOR color2, double hz) {
            for (Light light : lights) {
                light.flashing(color1, color2, hz);
            }
        }

        public void flashing(Constants.LED.COLOR color1, Constants.LED.COLOR color2, double hz, int repeat) {
            for (Light light : lights) {
                light.flashing(color1, color2, hz, repeat);
            }
        }

        public void flashingReset() {
            for (Light light : lights) {
                light.flashingReset();
            }
        }

        public void off() {
            for (Light light : lights) {
                light.off();
            }
        }

        public void lightsUpdate() {
            if (lightTimer.milliseconds() > 0) { // I changed this to 0 as a test because I think the rule of 2hz is intended to limit constantly flashing lights
                if (robotContainer.beamBreakToggleButton.releaseDuration() < 0.4 && robotContainer.spindexer.slotColor[robotContainer.spindexer.getCurrentIntakeSlot()] == Constants.Game.ARTIFACT_COLOR.PURPLE) {
                    if (Status.currentLightColor != Constants.LED.COLOR.VIOLET) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.VIOLET);
                        lightTimer.reset();
                    }
                } else if (robotContainer.beamBreakToggleButton.releaseDuration() < 0.4 && robotContainer.spindexer.slotColor[robotContainer.spindexer.getCurrentIntakeSlot()] == Constants.Game.ARTIFACT_COLOR.GREEN) {
                    if (Status.currentLightColor != Constants.LED.COLOR.GREEN) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.GREEN);
                        lightTimer.reset();
                    }
                } else if (Status.flywheelAtTargetSpeed && Status.flywheelToggle) {
                    if (Status.currentLightColor != Constants.LED.COLOR.AZURE) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.AZURE);
                        lightTimer.reset();
                    }
                } else if (Status.flywheelToggle) {
                    if (Status.currentLightColor != Constants.LED.COLOR.RED) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.RED);
                        lightTimer.reset();
                    }
                } else if (Status.intakeGamepadable) {
                    if (Status.currentLightColor != Constants.LED.COLOR.ORANGE) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.ORANGE);
                        lightTimer.reset();
                    }
                } else {
                    if (Status.currentLightColor != Constants.LED.COLOR.YELLOW) {
                        robotContainer.allIndicatorLights.setColor(Constants.LED.COLOR.YELLOW);
                        lightTimer.reset();
                    }
                }
            }
        }

        public List<Light> getLights() {
            return lights;
        }
    }

    public static class Light {
        private final RobotContainer robotContainer;
        private final ServoImplEx indicatorLight;
        private final ElapsedTime flashingTimer = new ElapsedTime();
        private int count = 1;
        private double rainbowValue = 0;
        private int rainbowSign = 1;
        private final ElapsedTime rainbowTimer = new ElapsedTime();

        public Light(RobotContainer robotContainer, ServoImplEx indicatorLight) {
            this.robotContainer = robotContainer;
            this.indicatorLight = indicatorLight;
        }

        public void setColor(Constants.LED.COLOR color) {
            indicatorLight.setPosition(Objects.requireNonNull(Constants.LED.COLOR_MAP.get(color)).ANALOG);
        }

        public void rainbow() {
            if (rainbowTimer.milliseconds() >= 2) {
                rainbowValue += (0.001 * rainbowSign);
                rainbowTimer.reset();
                if (rainbowValue >= 1) {
                    rainbowSign = -1;
                } else if (rainbowValue <= 0) {
                    rainbowSign = 1;
                }
            }
            indicatorLight.setPosition(scalePosition(rainbowValue));
        }

        public void rainbowReset() {
            rainbowValue = 0;
            rainbowSign = 1;
            rainbowTimer.reset();
        }

        public void flashing(Constants.LED.COLOR color, Constants.LED.COLOR colorTwo, double hz) {
            if (flashingTimer.seconds() <= 1 / hz) {
                setColor(color);
            } else if (flashingTimer.seconds() <= 2 / hz) {
                setColor(colorTwo);
            } else {
                flashingTimer.reset();
            }
        }

        public void flashing(Constants.LED.COLOR color, Constants.LED.COLOR colorTwo, double hz, int timesRepeated) {
            if (flashingTimer.seconds() <= 1 / hz) {
                setColor(color);
            } else if (flashingTimer.seconds() <= 2 / hz) {
                setColor(colorTwo);
            } else if (!(count >= timesRepeated)) {
                flashingTimer.reset();
                count += 1;
            }
        }

        public void flashingReset() {
            flashingTimer.reset();
            count = 1;
        }

        public void off() {
            setColor(Constants.LED.COLOR.OFF);
        }

        public double scalePosition(double position) {
            return (
                    position *
                            (Objects.requireNonNull(Constants.LED.COLOR_MAP.get(Constants.LED.COLOR.VIOLET)).ANALOG -
                                    Objects.requireNonNull(Constants.LED.COLOR_MAP.get(Constants.LED.COLOR.RED)).ANALOG)) +
                    Objects.requireNonNull(Constants.LED.COLOR_MAP.get(Constants.LED.COLOR.RED)).ANALOG;
        }
    }
}
