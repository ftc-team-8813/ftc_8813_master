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
    int add, over;
    Config c;
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
        c = BaseAutonomous.instance().config;
        move = c.getInt("toJewel_" + quadrant, 1);
        //TODO add to config
        add = c.getInt("knock_dist", 1000);
        over = c.getInt("over_dist", 500);
        if (isBlue) {
            add = -add;
            //over = over;
        }
    }
    @Override
    public void runTask() throws InterruptedException {
        log.i("Started");
        base.runToPosition(move + over, false);
        int isRed = 0;
        // up, mid, down

        double[] armPos = c.getDoubleArray("color_arm_positions");
        colorArm.setPosition(armPos[2]);
        sleep(800);
        if(colorSensor.red() > colorSensor.blue()){
            isRed = 1;
        } else if (colorSensor.blue() > colorSensor.red()){
            isRed = 2;
        } else {
            log.w("Unable to detect blue or red (detected r=%d,g=%d,b=%d", colorSensor.red(),
                    colorSensor.green(), colorSensor.blue());
        }
        TelemetryWrapper.setLine(3, String.valueOf(isRed));
        colorArm.setPosition(armPos[1]);
        sleep(800);
        base.runToPosition(move, false);
        colorArm.setPosition(armPos[3]);
        sleep(800);
        /*
            isBlue: Variable for what side of the field we are on.
            isRed:  Variable for color of the jewel. 1 is true, 2 is false
         */
        if(isRed == 1){
            base.startRunToPosition(move - add); //Turn right
        }else if(isRed == 2){
            base.startRunToPosition(move + add); //Turn left
        }
        sleep(500); //Not sure if needed
        colorSensor.enableLed(false);
        colorArm.setPosition(armPos[1]);
        log.i("Finished");
    }
}