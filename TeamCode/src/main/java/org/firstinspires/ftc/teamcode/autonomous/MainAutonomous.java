package org.firstinspires.ftc.teamcode.autonomous;


import android.annotation.SuppressLint;
import android.graphics.BitmapFactory;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskPlaceGlyphAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRotate;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskScoreJewel;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.util.Logger;

/**
 * Main autonomous program.
 */
@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous {
    /** Set to true if a color sensor is available and the jewel knocker should run */
    private static final boolean COLOR_SENSOR = false;

    //public abstract boolean isBlue();
    private volatile int quadrant;
    public int quadrant() {
        return quadrant;
    }
    public boolean find;
    private TaskClassifyPictograph finder;
    private Arm arm;
    private MotorController base;
    private MotorController extend;
    private Servo colorArm;
    private ColorSensor colorSensor;
    private Logger log;

    @Override
    public void initialize() {
        log = new Logger("Autonomous");
        log.v("Initializing autonomous program");
        find = config.getBoolean("run_finder", false);
        Servo ws = hardwareMap.servo.get("s0");
        Servo ss = hardwareMap.servo.get("s1");
        Servo es = hardwareMap.servo.get("s2");
        Servo claw = hardwareMap.servo.get("s3");
        Servo wrist = hardwareMap.servo.get("s4");
        Servo yaw = hardwareMap.servo.get("s5");
        colorArm = hardwareMap.servo.get("s6");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        arm = new Arm(ws, ss, es, claw, wrist, yaw);
        arm.closeClaw();
        DcMotor motor = hardwareMap.dcMotor.get("base");
        if (config.getBoolean("base_reverse", false))
            motor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor motor2 = hardwareMap.dcMotor.get("extend");
        if (config.getBoolean("ext_reverse", false))
            motor.setDirection(DcMotorSimple.Direction.REVERSE);

        base = new MotorController(motor);
        extend = new MotorController(motor2);
        //moveArm(.4134, .1303, .05);
        //Same as TeleOp
        ArmDriver driver = new ArmDriver(arm, config.getDouble("l1", 1),
                config.getDouble("l2", 1), config);
        claw.setPosition(config.getDouble("claw_part", 0));
        //claw_closed = true;
        driver.moveTo(config.getDouble("dist_init", 0),
                config.getDouble("adj_init", 0));
        driver.setWaistAngle(config.getDouble("waist_init", 0));
        wrist.setPosition(config.getDouble("wrist_init", 0));
        chooseQuadrant();
        finder = new TaskClassifyPictograph();
    }

    private void chooseQuadrant() {
        telemetry.addData("","Please choose a quadrant on the robot controller");
        telemetry.update();
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance()
                .getActivity();
        final int center_x = 210;
        final int center_y = 244;
        final int w = 421, h = 559;
        final Logger log = new Logger("Quadrant Chooser");
        activity.runOnUiThread(new Runnable() {
            @SuppressLint("ClickableViewAccessibility")
            @Override
            public void run() {
                ImageView image = new ImageView(activity);
                image.setImageBitmap(BitmapFactory.decodeResource(activity.getResources(), R
                        .drawable.field));
                activity.cameraMonitorLayout.addView(image);
                image.setOnTouchListener(new View.OnTouchListener() {
                    @Override
                    public boolean onTouch(View v, MotionEvent event) {
                        if (event.getAction() == MotionEvent.ACTION_DOWN) {
                            int touch_x = (int)event.getX();
                            int touch_y = (int)event.getY();
                            log.d("Received touch event at (%d, %d)", touch_x, touch_y);
                            int dx = (int)-v.getX();
                            int dy = (int)-v.getY();
                            int vw = (int)v.getWidth();
                            int vh = (int)v.getHeight();
                            int px = (touch_x + dx) * w / vw;
                            int py = (touch_y + dy) * h / vh;
                            log.d("Click event at (%d,%d); transformed to image pixel (%d, %d)",
                                    touch_x, touch_y, px, py);
                            if (px < center_x) {
                                if (py < center_y) {
                                    quadrant = 1;
                                } else {
                                    quadrant = 2;
                                }
                            } else {
                                if (py < center_y) {
                                    quadrant = 4;
                                } else {
                                    quadrant = 3;
                                }
                            }
                            activity.cameraMonitorLayout.removeView(v);
                            return true;
                        }
                        return false;
                    }
                });
                log.i("Listening for click events");

            }
        });
        quadrant = 0;
        while (quadrant == 0) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                return;
            }
        }
        log.i("Quadrant chosen: " + quadName(quadrant));
        telemetry.clear();
        telemetry.addData("Quadrant chosen", quadName(quadrant));
    }

    @Override
    public void run() throws InterruptedException {
        TelemetryWrapper.clear();
        //Run finder if enabled
        if (find) {
            log.i("Rotating to pictograph");
            tasks.add(new TaskRotate(base, config.getInt("toPict_" + quadrant(), 0)));
            log.i("Detecting pictograph");
            finder.addTask(new TaskScoreJewel(quadrant(), base, colorArm, colorSensor));
            tasks.add(finder);
            runTasks();
        }
        //Get result if finder is not null
        TaskClassifyPictograph.Result result = finder == null ? null : finder.getResult();
        //Set result to NONE if result is null
        if (result == null) result = TaskClassifyPictograph.Result.NONE;
        //Place glyph
        log.i("Placing glyph");
        tasks.add(new TaskPlaceGlyphAutonomous(quadrant(), result, base, extend, arm));
        //Knock jewel
    }

    @Override
    public void finish() throws InterruptedException {
        //All motor controllers need to be closed or strange bugs will crop up
        base.close();
        extend.close();
    }

    private String quadName(int quadrant) {
        switch (quadrant) {
            case 1: return "Blue Upper";
            case 2: return "Blue Lower";
            case 3: return "Red Lower";
            case 4: return "Red Upper";
            default: return "Unknown";
        }
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