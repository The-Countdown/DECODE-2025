package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class HuskyLensLogic {
    private final RobotContainer robotContainer;

    private final HuskyLens huskyLens;

    public enum Color {
        GREEN,
        PURPLE,
        NONE
    }
    public HuskyLensLogic(RobotContainer robotContainer, HuskyLens lens) {
        this.robotContainer = robotContainer;
        this.huskyLens = lens;
    }

    public Color checkColor() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if(blocks.length == 0){
            robotContainer.telemetry.addData("Seen ball: ", Color.NONE);
            return Color.NONE;
        }else {
            if (blocks[0].id == 2) {
                robotContainer.telemetry.addData("Seen ball: ", Color.PURPLE);
                return Color.PURPLE;
            } else if (blocks[0].id == 1) {
                robotContainer.telemetry.addData("Seen ball: ", Color.GREEN);
                return Color.GREEN;
            } else {
                robotContainer.telemetry.addData("Seen ball: ", Color.NONE);
                return Color.NONE;
            }
        }
    }

    // 320 X left -> right
    // 240 Y top -> bottom
    private int getX(int rawX){
        return rawX - 160;
    }
    private int getY(int rawY){
        return Math.abs(rawY*-1+240);
    }
    public String nearestBall(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double distance;
        double nearestGreenDistance = 999;
        double nearestGreenX = 0;
        double nearestGreenY = 0;
        double nearestPurpleDistance = 999;
        double nearestPurpleX = 0;
        double nearestPurpleY = 0;

        if(blocks.length == 0){
            robotContainer.telemetry.addData("Seen ball: ", Color.NONE);
            return "Nearest ball: None";
        }else {
            for (HuskyLens.Block block : blocks) {
                if (block.id == 1) {
                    distance = Math.sqrt(Math.pow(getX(block.x), 2) + Math.pow(getY(block.y), 2));
                    if (distance < nearestGreenDistance) {
                        nearestGreenDistance = distance;
                        nearestGreenX = getX(block.x);
                        nearestGreenY = getY(block.y);
                    }
                } else if (block.id == 2) {
                    distance = Math.sqrt(Math.pow(getX(block.x), 2) + Math.pow(getY(block.y), 2));
                    if (distance < nearestPurpleDistance) {
                        nearestPurpleDistance = distance;
                        nearestPurpleX = getX(block.x);
                        nearestPurpleY = getY(block.y);
                    }
                }
            }
        }
        robotContainer.telemetry.addData("Nearest Green Distance: ", nearestGreenDistance);
        robotContainer.telemetry.addData("Nearest Green x: ", nearestGreenX);
        robotContainer.telemetry.addData("Nearest Green y: ", nearestGreenY);

        robotContainer.telemetry.addData("", "");

        robotContainer.telemetry.addData("Nearest Purple Distance: ", nearestPurpleDistance);
        robotContainer.telemetry.addData("Nearest Purple x: ", nearestPurpleX);
        robotContainer.telemetry.addData("Nearest Purple y: ", nearestPurpleY);
        robotContainer.telemetry.update();

        return "Nearest ball: " + blocks[0].x + " , " + blocks[0].y + " , " + checkColor();
    }

    public void trackBall(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double turnPower = 0.1;

        //I need these two
        double currentAngle = 0;
        double degreesPerX = 0.1; // The amount of degrees needed to move the x value by 1

        double targetAngle = currentAngle;

        if(blocks.length > 0){
            double x = blocks[0].x;
            if(x > 10){
                targetAngle = ((Math.PI/180)-1)*(currentAngle + ((x-10) * degreesPerX));
            } else if (x < -10){
                targetAngle = ((Math.PI/180)-1)*(currentAngle - ((x+10) * degreesPerX));
            }
            double[] targets = {targetAngle,targetAngle,targetAngle,targetAngle};
            double[] powers = {turnPower,turnPower,turnPower,turnPower};
            robotContainer.drivetrain.setTargets(targets, powers);
        }
    }

    // GOAL: return the nearest green and the nearest purple ball
    // * Get telemetry to return position and color of nearest ball
    // * create for loop to get distance of each ball
    // * create if statement to check the color of that ball  if it is closer than the current nearest ball of that color and set it as new nearest ball
    // * return the nearest green and nearest purple ball with x, y, and distance for both
    // * set 0, 0 to be at bottom center of the cameras vision aka at the intake (offset x and y coords)
    // x if reliable enough set the distance away from the robot based on the size of the box (farther away = smaller block width/height)
    // x get the camera to correctly update the telemetry (right now it just shows "Nearest ball: None" and every second flashes the correct values)

}