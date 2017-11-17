package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

import static java.lang.Thread.sleep;

/**
 * Program that takes an integer to reflect what quadrant the robot is in
 * and places a block.
 *     -----------------
 *     |Quad 1 | Quad 2|
 *     |       |       |
 *     ----------------|
 *     |Quad 3 | Quad 4|
 *     |       |       |
 *     -----------------
 *      Relic     Relic
 */

public class TaskPlaceGlyphAutonomous implements Task{
    private Servo ax, ay, el, cw;
    private DcMotor rt, ex;
    private TouchSensor lm;
    private int quadrant;
    private Task task;
    public TaskPlaceGlyphAutonomous(int quadrant) {
        this.quadrant = quadrant;
        HardwareMap m = BaseAutonomous.instance().hardwareMap;
        ax = m.servo.get("s0"); //waist
        ay = m.servo.get("s1"); //elbow
        el = m.servo.get("s2"); //shoulder
        cw = m.servo.get("s3");
        rt = m.dcMotor.get("base");
        ex = m.dcMotor.get("extend");
        lm = m.touchSensor.get("ext_bumper");
    }
    @Override
    public void runTask() throws InterruptedException {
        switch(quadrant){
            case 1: moveArm(.3486, .1330, .0691);
                    cw.setPosition(0);
                    sleep(5000);
                    moveArm(.2523, .2486, .3356);
                    sleep(5000);
                    moveArm(.2523, .5042, .4364);
                    sleep(5000);
                    cw.setPosition(1);
                    sleep(500);
                    moveArm(.2065, .5042, .2821);
                    sleep(5000);
                    break;
            /*case 2: ax.setPosition();
                    ay.setPosition();
                    break;
            case 3: ax.setPosition();
                    ay.setPosition();
                    break;
            case 4: ax.setPosition();
                    ay.setPosition();
                    break;
            default:break;*/
        }
    }   /**
            A simple method that takes arm positions and moves the arm. Need to add wait function.
        **/
        private void moveArm(double waist, double elbow, double shoulder){
            ax.setPosition(waist);
            ay.setPosition(elbow);
            el.setPosition(shoulder);
    }
}/*ax.setPosition(.2523);
                    ay.setPosition(.3879);
                    el.setPosition(.3239);*/