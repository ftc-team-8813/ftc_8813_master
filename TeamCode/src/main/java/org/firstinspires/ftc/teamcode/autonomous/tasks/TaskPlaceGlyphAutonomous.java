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
    Config move;


    public TaskPlaceGlyphAutonomous(int quadrant, TaskClassifyPictograph.Result result, MotorController base, Arm arm) {
        this.arm = arm;
        this.base = base;
        this.result = result;
        this.quadrant = quadrant;
        Config c = BaseAutonomous.instance().config;
        move = new Config(c.getString("pos_quadrant_" + quadrant, ""));
    }

    @Override
    public void runTask() throws InterruptedException {
        double waist;
        double elbow;
        double shoulder;
        if (result == TaskClassifyPictograph.Result.NONE) {
            result = TaskClassifyPictograph.Result.CENTER;
        }
        TelemetryWrapper.setLines(15);
        TelemetryWrapper.setLine(0, "Result: " + result.name());

//        double[] vals = move.getDoubleArray("init_move");
//        if (vals == null) {
//            TelemetryWrapper.setLine(1, "No init_move data!");
//            return;
//        }
//        moveArm(vals);
//        TelemetryWrapper.setLine(1, "Moving to start position");
//        sleep(2000);

        move("floating");
        sleep(2500);
        move("move_" + result.name());
        sleep(4000);
        arm.openClaw();
        move("safe_" + result.name());
        sleep(4000);
        move("parking");
        sleep(4000);
        /*
        move_LEFT: 0.4480, 0.5029, 0.1929, 0.42, -1085
safe_LEFT: 0.4365, 0.4613, 0.2201, 0, -1085
move_CENTER: 0.4551, 0.5932, 0.3210, 0.1985, -1085
safe_CENTER: 0.4458, 0.5610, 0.3210, 0.1985, -1085
move_RIGHT: 0.4462, 0.6059, 0.3210, 0.1158, -1085
safe_RIGHT: 0.4462, 0.5417, 0.3210, 0.1158, -1085
parking: 0.3908, 0.5534, 0.2411, 0, -1085
start: 0.3934, 0.1108, 0.3207, 0.42, 0
pictograph: 0.3878, 0.1745, .2943, 0.8912, -2979
floating: 0.3937, 0.4810, 0.2369, 0.42, -1085
         */
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

    private void move(String values){
        double[] vals = move.getDoubleArray(values);
        if (vals == null) {
            TelemetryWrapper.setLine(5, "No "+ values + " data!");
            return;
        }else {
            moveArm(vals);
            TelemetryWrapper.setLine(6, "Moving to " + values);
        }
    }
}