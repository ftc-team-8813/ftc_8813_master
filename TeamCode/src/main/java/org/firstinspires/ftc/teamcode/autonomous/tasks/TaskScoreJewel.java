package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import static java.lang.Thread.sleep;


/**
 * Created by Joseph Murphy on 12/1/2017
 */

public class TaskScoreJewel implements Task{
    MotorController base;
    Servo colorArm;
    boolean isBlue;
    ColorSensor colorSensor;
    private Logger log;
    int move;
    public TaskScoreJewel(int quadrant, MotorController base, Servo colorArm, ColorSensor colorSensor){
        log = new Logger("Jewel Scorer");
        this.base = base;
        this.colorArm = colorArm;
        this.colorSensor = colorSensor;
        colorSensor.enableLed(true);
        if(quadrant == 2 || quadrant == 1){
            isBlue = true;
        }else{
            isBlue = false;
        }
        Config c = BaseAutonomous.instance().config;
        move = c.getInt("toJewel_" + quadrant, 1);
    }
    @Override
    public void runTask() throws InterruptedException {
        log.i("Started");
        base.hold(move);
        sleep(2000);
        int isRed = 0;
        colorArm.setPosition(BaseAutonomous.instance().config.getDouble("color_arm_down", .6305));
        sleep(2000);
        if(colorSensor.red() > colorSensor.blue()){
            isRed = 1;
        }else if (colorSensor.blue() > colorSensor.red()){
            isRed = 2;
        }
        TelemetryWrapper.setLine(3, String.valueOf(isRed));
        /*
            isBlue: Variable for what side of the field we are on.
            isRed:  Variable for color of the jewel. 1 is true, 2 is false
         */
        if(isBlue){
            if(isRed == 1){
                base.hold(move + 1000); //Turn right
            }else if(isRed == 2){
                base.hold(move - 1000); //Turn left
            }else{}
        }else{
            if(isRed == 2){
                base.hold(move + 1000); //Knock off red
            }else if (isRed == 1){
                base.hold(move - 1000); //Knock off blue
            }else{}
        }
        sleep(500); //Not sure if needed
        colorSensor.enableLed(false);
        colorArm.setPosition(0.1);
        log.i("Finished");
    }
}