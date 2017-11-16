package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

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
            case 1: ax.setPosition(.3486);
                    ay.setPosition(.1330);
                    el.setPosition(.0691);
                    cw.setPosition(0);
                    Thread.sleep(5000);
                    ax.setPosition(.2523);
                    ay.setPosition(.2486);
                    el.setPosition(.3356);
                    Thread.sleep(5000);
                    ax.setPosition(.2523);
                    ay.setPosition(.5042);
                    el.setPosition(.4364);
                    Thread.sleep(5000);
                    cw.setPosition(1);
                    Thread.sleep(500);
                    ax.setPosition(.2523);
                    ay.setPosition(.3879);
                    el.setPosition(.3239);
                    Thread.sleep(5000);
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
    }/*ax.setPosition(.2673);
                    ay.setPosition(.2486);
                    el.setPosition(.2568);*/
}