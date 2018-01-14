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
        TelemetryWrapper.setLines(2);
        TelemetryWrapper.setLine(0, "Result: " + result.name());

        Config move = new Config(c.getString("move_quad_" + quadrant, ""));

        base.runToPosition(0);
//        double[] vals = move.getDoubleArray("init_move");
//        if (vals == null) {
//            TelemetryWrapper.setLine(1, "No init_move data!");
//            return;
//        }
//        moveArm(vals);
//        TelemetryWrapper.setLine(1, "Moving to start position");
//        sleep(2000);

        double[] vals = move.getDoubleArray("move_" + result.name());
        if (vals == null) {
            TelemetryWrapper.setLine(1, "No move_" + result.name() + " data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(1, "Moving to key column");
        sleep(4000);
        arm.openClaw();

        vals = move.getDoubleArray("park");
        if (vals == null) {
            TelemetryWrapper.setLine(1, "No park data!");
            return;
        }
        moveArm(vals);
        TelemetryWrapper.setLine(1, "Moving to park position");
        sleep(1000);

    }

    /**
     * A simple method that takes arm positions and moves the arm and turntable.
     **/
    private void moveArm(double waist, double shoulder, double elbow, double wrist, double rotate) {
        arm.moveTo(waist, shoulder, elbow);
        arm.moveWrist(wrist);
        base.hold((int)rotate);
    }

    private void moveArm(double... pos) {
        moveArm(pos[0], pos[1], pos[2], pos[3], pos[4]);
    }
}