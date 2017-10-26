package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectJewel;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDoMove;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskExtendSlide;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRunServo;
import org.firstinspires.ftc.teamcode.autonomous.util.Config;

import java.util.Arrays;

/**
 * Main autonomous program. RedAutonomous and BlueAutonomous are the sub-OpModes that decide whether
 * isRed or isBlue is true.
 */

public abstract class MainAutonomous extends BaseAutonomous {

    private static final boolean COLOR_SENSOR = false;

    public abstract boolean isBlue();

    protected TaskClassifyPictograph finder;
    @Override
    public void initialize() {
        finder = new TaskClassifyPictograph();
    }

    @Override
    public void run() throws InterruptedException {
        tasks.add(finder);
        tasks.add(new TaskExtendSlide(hardwareMap.dcMotor.get("extend")));
        tasks.add(new TaskDoMove("to_jewels.dat"));
        TaskDetectJewel jd = null;
        if (COLOR_SENSOR) {
            jd = new TaskDetectJewel(hardwareMap.colorSensor.get("color"));
            tasks.add(jd);
        }
        runTasks();
        if (COLOR_SENSOR) {
            if (jd.getReading() == TaskDetectJewel.RED) {
                if (isBlue())
                    tasks.add(new TaskDoMove("jewel_blue.dat"));
                else
                    tasks.add(new TaskDoMove("jewel_red.dat"));
            } else if (jd.getReading() == TaskDetectJewel.BLUE) {
                if (isBlue())
                    tasks.add(new TaskDoMove("jewel_red.dat"));
                else
                    tasks.add(new TaskDoMove("jewel_blue.dat"));
            } else {
                telemetry.addData("Color detected was not valid", Arrays.toString(jd.getDetectedColor()));
                telemetry.update();
            }
        }
        if (finder.getResult().equals(TaskClassifyPictograph.Result.LEFT)) {
            tasks.add(new TaskDoMove("place_block_l.dat"));
        } else if (finder.getResult().equals(TaskClassifyPictograph.Result.CENTER)) {
            tasks.add(new TaskDoMove("place_block_c.dat"));
        } else {
            tasks.add(new TaskDoMove("place_block_r.dat"));
        }
    }
}

abstract class BasicAutonomous extends MainAutonomous {
    @Override
    public void run() throws InterruptedException {
        Config cf = BaseAutonomous.instance().config;
        Servo w = hardwareMap.servo.get("s0");
        Servo s = hardwareMap.servo.get("s1");
        Servo e = hardwareMap.servo.get("s2");
        Servo c = hardwareMap.servo.get("s3");
        tasks.add(new TaskRunServo(c, cf.getDouble("claw_closed",0)));
        tasks.add(finder);
        runTasks();
        if (isBlue()) {
            /////////////////////
            //BLUE CODE HERE!!!//
            /////////////////////

            //Go to the relic zone
            if (finder.getResult().equals(TaskClassifyPictograph.Result.LEFT)) {
                tasks.add(new TaskRunServo(w, cf.getDouble("blue.left.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("blue.left.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("blue.left.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("blue.left.delay",0)));
            } else if (finder.getResult().equals(TaskClassifyPictograph.Result.CENTER)) {
                tasks.add(new TaskRunServo(w, cf.getDouble("blue.center.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("blue.center.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("blue.center.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("blue.center.delay",0)));
            } else {
                tasks.add(new TaskRunServo(w, cf.getDouble("blue.right.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("blue.right.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("blue.right.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("blue.right.delay",0)));
            }
            //Drop the block
            tasks.add(new TaskRunServo(c, cf.getDouble("claw_open",0)));
            //Pull out so we can fall into the parking zone
            tasks.add(new TaskDelay(0));
            tasks.add(new TaskRunServo(w, cf.getDouble("blue.pullout.waist",0)));
            tasks.add(new TaskRunServo(s, cf.getDouble("blue.pullout.shoulder",0)));
            tasks.add(new TaskRunServo(e, cf.getDouble("blue.pullout.elbow",0)));
        } else {
            /////////////////
            //RED CODE HERE//
            /////////////////

            //Go to the relic zone
            if (finder.getResult().equals(TaskClassifyPictograph.Result.LEFT)) {
                tasks.add(new TaskRunServo(w, cf.getDouble("red.left.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("red.left.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("red.left.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("red.left.delay",0)));
            } else if (finder.getResult().equals(TaskClassifyPictograph.Result.CENTER)) {
                tasks.add(new TaskRunServo(w, cf.getDouble("red.center.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("red.center.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("red.center.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("red.center.delay",0)));
            } else {
                tasks.add(new TaskRunServo(w, cf.getDouble("red.right.waist",0)));
                tasks.add(new TaskRunServo(s, cf.getDouble("red.right.shoulder",0)));
                tasks.add(new TaskRunServo(e, cf.getDouble("red.right.elbow",0)));
                tasks.add(new TaskDelay(cf.getInt("red.right.delay",0)));
            }
            //Drop the block
            tasks.add(new TaskRunServo(c, cf.getDouble("claw_open",0)));
            //Pull out so we can fall into the parking zone
            tasks.add(new TaskDelay(0));
            tasks.add(new TaskRunServo(w, cf.getDouble("red.pullout.waist",0)));
            tasks.add(new TaskRunServo(s, cf.getDouble("red.pullout.shoulder",0)));
            tasks.add(new TaskRunServo(e, cf.getDouble("red.pullout.elbow",0)));
        }
    }
}

