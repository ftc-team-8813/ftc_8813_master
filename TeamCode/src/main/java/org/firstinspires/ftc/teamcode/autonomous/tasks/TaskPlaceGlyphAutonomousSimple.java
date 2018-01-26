package org.firstinspires.ftc.teamcode.autonomous.tasks;

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

public class TaskPlaceGlyphAutonomousSimple implements Task {
    private int quadrant;
    private Arm arm;
    private TaskClassifyPictograph.Result result;
    private MotorController base;


    public TaskPlaceGlyphAutonomousSimple(int quadrant, TaskClassifyPictograph.Result result, MotorController base, Arm arm) {
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
        switch(quadrant){
            case(1):
                break;
            case(2):
                break;
            case(3):
                break;
            case(4):
                moveArm( 0.30655508403478265, .75, 0.28754293442759327, 0.6900000000000002, -11.0);
                sleep(5000);
                /*moveArm(0.3779, 0.2559, 0.4440, 0.5227, 6018);
                sleep(2000);
                moveArm(0.3655, 0.2963, 0.5638, 0.42, 5804);
                sleep(2000);*/
        }
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