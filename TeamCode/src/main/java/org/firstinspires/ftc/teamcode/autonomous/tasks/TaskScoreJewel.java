package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;

import static java.lang.Thread.sleep;


/**
 * Created by Joseph Murphy on 12/1/2017
 */

public class TaskScoreJewel implements Task{
    MotorController base;
    Servo colorArm;
    boolean isBlue;
    ColorSensor colorSensor;
    public TaskScoreJewel(int quadrant, MotorController base, Servo colorArm, ColorSensor colorSensor){
        this.base = base;
        this.colorArm = colorArm;
        this.colorSensor = colorSensor;
        colorSensor.enableLed(true);
        if(quadrant == 2 || quadrant == 4){
            isBlue = false;
        }else{
            isBlue = true;
        }
    }
    @Override
    public void runTask() throws InterruptedException {
        base.hold(1); //FIND POSITION
        sleep(2000);
        boolean isRed = false;
        colorArm.setPosition(.75); //MUST CHANGE
        if(colorSensor.red() > colorSensor.blue()){
            isRed = true;
        }else if (colorSensor.blue() > colorSensor.red()){
            isRed = false;
        }
        /*
            isBlue: Variable for what side of the field we are on.
            isRed:  Variable for color of the jewel.
         */
        if(isBlue){
            if(isRed == true){
                base.hold(2039); //Knock off red
            }else{
                base.hold(9090); //Knock off blue
            }
        }else{
            if(isRed == false){
                base.hold(8734); //Knock off red
            }else{
                base.hold(5837); //Knock off blue
            }
        }
        sleep(500); //Not sure if needed
        colorArm.setPosition(0);
        colorSensor.enableLed(false);
    }
}