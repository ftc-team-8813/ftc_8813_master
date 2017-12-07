package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.Config;

import static java.lang.Thread.sleep;

/**
 * Program that takes an integer to reflect what quadrant the robot is in
 * and places a block.
 * -----------------
 * |Quad 1 | Quad 2|
 * |       |       |
 * ----------------|
 * |Quad 3 | Quad 4|
 * |       |       |
 * -----------------
 * Relic     Relic
 */

public class TaskPlaceGlyphAutonomous implements Task {
    private Servo ax, ay, el, cw;
    private DcMotor rt, ex;
    private TouchSensor lm;
    private int quadrant;
    private Task task;
    private TaskClassifyPictograph.Result result;


    public TaskPlaceGlyphAutonomous(int quadrant, TaskClassifyPictograph.Result result) {
        this.result = result;
        this.quadrant = quadrant;
        HardwareMap m = BaseAutonomous.instance().hardwareMap;
        ax = m.servo.get("s0"); //waist
        ay = m.servo.get("s1"); //elbow
        el = m.servo.get("s2"); //shoulder
        cw = m.servo.get("s3"); //claw
        rt = m.dcMotor.get("base");
        ex = m.dcMotor.get("extend");
        lm = m.touchSensor.get("ext_bumper");
    }

    @Override
    public void runTask() throws InterruptedException {
        Config c = BaseAutonomous.instance().config;
        double waist;
        double elbow;
        double shoulder;
        if (result == TaskClassifyPictograph.Result.NONE) {
            result = TaskClassifyPictograph.Result.CENTER;
        }

        moveArm(c.getDouble("w_i", 0),
                c.getDouble("s_i", 0),
                c.getDouble("e_i", 0));
        sleep(2000);

        int columnN = result.ordinal();
        moveArm(c.getDouble("w_"+(quadrant-1)+""+columnN, 0),
                c.getDouble("s_"+(quadrant-1)+""+columnN, 0),
                c.getDouble("e_"+(quadrant-1)+""+columnN, 0));

        sleep(3000);
        cw.setPosition(c.getDouble("claw_open", 0));

        moveArm(c.getDouble("wp_" + (quadrant-1), 0),
                c.getDouble("sp_" + (quadrant-1), 0),
                c.getDouble("ep_" + (quadrant-1), 0));
        sleep(1000);

    }

    /**
     * A simple method that takes arm positions and moves the arm. Need to add wait function.
     **/
    private void moveArm(double waist, double shoulder, double elbow) {
        ax.setPosition(waist);
        ay.setPosition(shoulder);
        el.setPosition(elbow);
    }
}