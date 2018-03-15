package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import static java.lang.Thread.sleep;

/**
 * Program that takes an integer to reflect what quadrant the robot is in
 * and places a block.
 * -----------------
 * |Quad 1 | Quad 4|
 * |       |       |
 * ----------------|
 * |Quad 2 | Quad 3|
 * |       |       |
 * -----------------
 * Relic     Relic
 */

public class TaskPlaceGlyphAutonomous implements Task {
    private int quadrant;
    private Arm arm;
    private TaskClassifyPictograph.Result result;
    private MotorController base;
    private MotorController extend;
    private Logger log = new Logger("Glyph Placer");
    Config move;


    public TaskPlaceGlyphAutonomous(int quadrant, TaskClassifyPictograph.Result result,
                                    MotorController base, MotorController extend, Arm arm) {
        this.arm = arm;
        this.base = base;
        this.extend = extend;
        this.result = result;
        this.quadrant = quadrant;
        Config c = BaseAutonomous.instance().config;
        move = new Config(c.getString("pos_quadrant_" + quadrant, ""));
    }

    @Override
    public void runTask() throws InterruptedException {
        if (result == TaskClassifyPictograph.Result.NONE) {
            result = TaskClassifyPictograph.Result.CENTER;
        }
        TelemetryWrapper.setLines(15);
        TelemetryWrapper.setLine(0, "Result: " + result.name());

        move("floating");
        sleep(2500);
        move("floating_extended");
        sleep(2500);
        move("move_" + result.name());
        sleep(4000);
        arm.openClaw();
        move("safe_" + result.name());
        sleep(4000);
        move("parking");
        sleep(4000);
    }


    /**
     * A simple method that takes arm positions and moves the arm and turntable.
     **/
    private void moveArm(double waist, double shoulder, double elbow, double wrist, double yaw,
                         double rotate, double extension) {
        log.i("Moving to waist: %.4f, shoulder: %.4f, elbow: %.4f, wrist: %.4f, yaw: %.4f " +
                "rotation: %d, " + "extend: %d", waist, shoulder, elbow, wrist, yaw, (int)rotate,
                (int)extension);
        arm.moveTo(waist, shoulder, elbow);
        arm.moveWrist(wrist);
        arm.moveYaw(yaw);
        base.hold((int)rotate);
        extend.hold((int)extension);
    }

    private void moveArm(double... pos) {
        moveArm(pos[0], pos[1], pos[2], pos[3], pos[6], pos[4], pos[5]);
    }

    private void move(String values){
        double[] vals = move.getDoubleArray(values);
        if (vals == null) {
            TelemetryWrapper.setLine(5, "No "+ values + " data!");
            return;
        }else {
            TelemetryWrapper.setLine(6, "Moving to " + values);
            moveArm(vals);
        }
    }
}
