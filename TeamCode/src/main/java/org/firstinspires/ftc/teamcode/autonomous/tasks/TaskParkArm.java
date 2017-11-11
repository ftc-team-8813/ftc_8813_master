package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

/**
 * Program that takes an integer to reflect what quadrant the robot is in
 * and places a block. This is not good programming and should be replaced
 * for the next league meet!
 *     -----------------
 *     |Quad 1 | Quad 2|
 *     |       |       |
 *     ----------------|
 *     |Quad 3 | Quad 4|
 *     |       |       |
 *     -----------------
 */

public class TaskParkArm implements Task{
    private Servo ax, ay, el, cw;
    private DcMotor rt, ex;
    private TouchSensor lm;
    private int quadrant;
    public TaskParkArm(int quadrant) {
        this.quadrant = quadrant;
        HardwareMap m = BaseAutonomous.instance().hardwareMap;
        ax = m.servo.get("s0");
        ay = m.servo.get("s1");
        el = m.servo.get("s2");
        cw = m.servo.get("s3");
        rt = m.dcMotor.get("base");
        ex = m.dcMotor.get("extend");
        lm = m.touchSensor.get("ext_bumper");}
    @Override
    public void runTask() throws InterruptedException {
     /*   switch(quadrant){
            case 1: ax.setPosition();
                    ay.setPosition();
                    break;
            case 2: ax.setPosition();
                    ay.setPosition();
                    break;
            case 3: ax.setPosition();
                    ay.setPosition();
                    break;
            case 4: ax.setPosition();
                    ay.setPosition();
                    break;
            default:break;
        }*/
    }
}
