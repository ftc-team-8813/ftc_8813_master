package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Logger log = new Logger("Glyph Placer");


    public TaskPlaceGlyphAutonomous(int quadrant, TaskClassifyPictograph.Result result, MotorController base, Arm arm) {
        this.arm = arm;
        this.base = base;
        this.result = result;
        this.quadrant = quadrant;

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
        TelemetryWrapper.setLines(15);
        TelemetryWrapper.setLine(0, "Result: " + result.name());

        Config move = new Config(c.getString("pos_quadrant_" + quadrant, ""));

        base.runToPosition(0);
//        double[] vals = move.getDoubleArray("init_move");
//        if (vals == null) {
//            TelemetryWrapper.setLine(1, "No init_move data!");
//            return;
//        }
//        moveArm(vals);
//        TelemetryWrapper.setLine(1, "Moving to start position");
//        sleep(2000);

        double[] vals = move.getDoubleArray("pictograph");
        TelemetryWrapper.setLine(1, vals.toString());
        if (vals == null) {
            TelemetryWrapper.setLine(1, "No pictograph data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(2, "Moving to pictograph");
        sleep(5000);

        vals = move.getDoubleArray("floating");
        if (vals == null) {
            TelemetryWrapper.setLine(3, "No floating data!");
            return;
        }
            moveArm(vals);
            TelemetryWrapper.setLine(4, "Moving to floating");
            sleep(5000);

        vals = move.getDoubleArray("floating_" + result.name());
        if (vals == null) {
            TelemetryWrapper.setLine(5, "No floating_" + result.name() + " data!");
            return;
        }else {
            moveArm(vals);
            TelemetryWrapper.setLine(6, "Moving to secondary floating!");
            sleep(5000);
        }

        vals = move.getDoubleArray("move_" + result.name());
        if (vals == null) {
            TelemetryWrapper.setLine(7, "No move_" + result.name() + " data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(8, "Moving to key column!");
        sleep(5000);
        arm.openClaw();

        vals = move.getDoubleArray("floating");
        if (vals == null) {
            TelemetryWrapper.setLine(9, "No floating data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(10, "Moving to floating again!");
        sleep(5000);

        vals = move.getDoubleArray("parking");
        if (vals == null) {
            TelemetryWrapper.setLine(11, "No park data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(12, "Moving to park position");
        sleep(5000);

    }

    /**
     * A simple method that takes arm positions and moves the arm and turntable.
     **/
    private void moveArm(double waist, double shoulder, double elbow, double wrist, double rotate) {
        log.i("Moving to waist: %.4f, shoulder: %.4f, elbow: %.4f, wrist: %.4f, rotation: %d", waist, shoulder, elbow, wrist, (int)rotate);
        arm.moveTo(waist, shoulder, elbow);
        arm.moveWrist(wrist);
        base.hold((int)rotate);
    }

    private void moveArm(double... pos) {
        moveArm(pos[0], pos[1], pos[2], pos[3], pos[4]);
    }
}