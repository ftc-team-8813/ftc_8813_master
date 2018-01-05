package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskPlaceGlyphAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRotate;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskScoreJewel;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;

/**
 * Main autonomous program.
 */

public abstract class MainAutonomous extends BaseAutonomous {

    private static final boolean COLOR_SENSOR = false;

    //public abstract boolean isBlue();
    public abstract int quadrant();
    public boolean find;
    private TaskClassifyPictograph finder;
    private Arm arm;
    private MotorController base;

    @Override
    public void initialize() {
        find = config.getBoolean("run_finder", false);
        Servo ws = hardwareMap.servo.get("s0");
        Servo ss = hardwareMap.servo.get("s1");
        Servo es = hardwareMap.servo.get("s2");
        Servo claw = hardwareMap.servo.get("s3");
        arm = new Arm(ws, ss, es, claw);
        arm.closeClaw();
        base = new MotorController(hardwareMap.dcMotor.get("base"));
        //moveArm(.4134, .1303, .05);
        ws.setPosition(.3863);
        ss.setPosition(.0378);
        es.setPosition(.0386);
        finder = new TaskClassifyPictograph();
    }

    @Override
    public void run() throws InterruptedException {
        if (find) {
            tasks.add(new TaskRotate(base, config.getInt("toPict_" + quadrant(), 0)));
            tasks.add(finder);
            runTasks();
        }
        TaskClassifyPictograph.Result result = finder == null ? null : finder.getResult();
        if (result == null) result = TaskClassifyPictograph.Result.NONE;
        tasks.add(new TaskPlaceGlyphAutonomous(quadrant(), result, base, arm));
        if (COLOR_SENSOR) tasks.add(new TaskScoreJewel(quadrant()));
    }
}
/*
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

            //Go to the cryptobox column
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
*/
