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

    private BaseAutonomous baseAutonomous;

    public TaskPlaceGlyphAutonomous(int quadrant, TaskClassifyPictograph.Result result) {
        this.result = result;
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
        double waist;
        double elbow;
        double shoulder;
        String column = result.name().toLowerCase();
        if (column.equals("none")) {
            column = "center";
        }
        switch (quadrant) {
            case 1:
                moveArm(.4134, .1303, .1386);
                cw.setPosition(0);
                sleep(5000);
                moveArm(.5398, .1818, .1660);
                sleep(2000);
                waist = BaseAutonomous.instance().config.getDouble("waist.blue." + column, 0);
                elbow = BaseAutonomous.instance().config.getDouble("elbow.blue." + column, 0);
                shoulder = BaseAutonomous.instance().config.getDouble("shoulder.blue" + column, 0);
                moveArm(waist, elbow, shoulder);
                sleep(5000);
                cw.setPosition(BaseAutonomous.instance().config.getDouble("claw_open", 0));
                sleep(3000);
                moveArm(waist, elbow, shoulder + .100);
                sleep(3000);
                break;

            case 2:
                moveArm(.4134, .1303, .1386);
                cw.setPosition(0);
                sleep(5000);
                moveArm(.5398, .1818, .1660);
                sleep(2000);
                waist = BaseAutonomous.instance().config.getDouble("waist.red." + column, 0);
                elbow = BaseAutonomous.instance().config.getDouble("elbow.red." + column, 0);
                shoulder = BaseAutonomous.instance().config.getDouble("shoulder.red." + column, 0);
                moveArm(waist, elbow, shoulder);
                sleep(5000);
                cw.setPosition(BaseAutonomous.instance().config.getDouble("claw_open", 0));
                sleep(3000);
                moveArm(waist, elbow, shoulder + .100);
                sleep(3000);
                break;

            case 3:
                moveArm(.4134, .1303, .1386);
                cw.setPosition(BaseAutonomous.instance().config.getDouble("claw_open", 0));
                sleep(5000);
                moveArm(.5398, .1818, .1660);
                sleep(2000);
                waist = .5721;
                elbow = .3197;
                shoulder = .3877;
                moveArm(waist, elbow, shoulder);
                sleep(5000);
                break;

            case 4:
                moveArm(.4134, .1303, .1386);
                cw.setPosition(BaseAutonomous.instance().config.getDouble("claw_open", 0));
                sleep(5000);
                moveArm(.5398, .1818, .1660);
                sleep(2000);
                waist = .4279;
                elbow = .3197;
                shoulder = .6123;
                moveArm(waist, elbow, shoulder);
                sleep(5000);
                break;

            default:
                break;
        }
    }

    /**
     * A simple method that takes arm positions and moves the arm. Need to add wait function.
     **/
    private void moveArm(double waist, double elbow, double shoulder) {
        ax.setPosition(waist);
        ay.setPosition(elbow);
        el.setPosition(shoulder);
    }
}