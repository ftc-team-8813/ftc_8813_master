package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.teleop.util.ServoAngleFinder;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.File;
import java.io.IOException;

/**
 * Main TeleOp control to control the {@link ArmDriver}.
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode {

    //The arm controller
    protected ArmDriver driver;
    //Claw servo
    protected Servo claw, wrist;
    //Drive motors
    protected MotorController base, extend;
    //Limit switch
//    protected DigitalChannel limit; //Not a digital channel anymore
    protected AnalogInput limit;
    //Extend motor minimum position
    protected Integer extMin = null;
    //Range of extend motor
    protected int extRange;
    //Configuration
    protected Config conf;

    protected ButtonHelper buttonHelper_1;
    //Maximum speed of arm servos (some arbitrary unit)
    private double maxMove;
    //Maximum amount of change allowed in 200ms second
    private double maxIncrease;
    //Maximum speed of waist servo (radians per 20ms)
    private double maxRotate;

    private double[] wristRange;
    private double wrist_speed;

    private double[] rotateWindow = new double[10];
    private double[] extWindow = new double[10];
    private int nextWindowSlotR = 0;
    private int nextWindowSlotE = 0;
    //Whether the claw open/close button is currently being held down
    private boolean aHeld;
    //Whether the claw is open or closed
    private boolean claw_closed;
    //The amount to close the claw
    private double clawCloseAmount;
    //The amount to open the claw
    private double clawOpenAmount;

    private int base_speed;
    private int ext_speed;

    private double l1;
    private double l2;
    private boolean robot1;

    private long start = 0;

    //protected int initEncoder;

    @Override
    public void init() {
        try {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        //If we're on robot 1, use robot 1 config
        if (hardwareMap.getAll(LynxModule.class).get(0).getSerialNumber().toString().equals
                ("DQ168FFD")) {
            conf = new Config("config.robot1.properties");
            robot1 = true;
            if (conf.readFailed()) {
                conf = new Config("config.properties"); // Use old config as a backup option
            }
        } else {
            conf = new Config("config.properties");
        }

        maxMove = conf.getDouble("max_move", 0.002);
        maxIncrease = conf.getDouble("max_inc", 0.02);
        maxRotate = conf.getDouble("max_rotate_speed", 0.02);
        wristRange = conf.getDoubleArray("wrist_range");
        wrist_speed = conf.getDouble("wrist_speed", 0.01);
        base_speed = conf.getInt("base_speed", 5);
        ext_speed = conf.getInt("ext_speed", 10);
        clawCloseAmount = conf.getDouble("claw_closed", 0);
        clawOpenAmount = conf.getDouble("claw_open", 0);
        l1 = conf.getDouble("l1", 1);
        l2 = conf.getDouble("l2", 1);

        buttonHelper_1 = new ButtonHelper(gamepad1);
        //Get motors and servos from hardware map
        Servo waist = hardwareMap.servo.get("s0");
        Servo shoulder = hardwareMap.servo.get("s1");
        Servo elbow = hardwareMap.servo.get("s2");
        wrist = hardwareMap.servo.get("s4");
        claw = hardwareMap.servo.get("s3");
        base = new MotorController(hardwareMap.dcMotor.get("base"));
        extend = new MotorController(hardwareMap.dcMotor.get("extend"));
        if (!robot1) limit = hardwareMap.analogInput.get("limit");
        //Initialize arm controller
        driver = new ArmDriver(waist, shoulder, elbow, l1, l2, conf);

        //Get extend motor range
        extRange = conf.getInt("ext_range", Integer.MAX_VALUE/2);

        if (conf.getBoolean("base_reverse", false))
            base.setReverse(true);
        if (conf.getBoolean("ext_reverse", false))
            extend.setReverse(true);

        if (wristRange != null) {
            wrist.scaleRange(wristRange[0], wristRange[1]);
        }

        //Set up
        setInitialPositions();
        ServoAngleFinder.create(hardwareMap);
    }

    protected int getTurntablePosition() {
        return base.getCurrentPosition();
    }

    private void setInitialPositions() {
        claw.setPosition(conf.getDouble("claw_closed", 0));
        claw_closed = true;
        driver.moveTo(conf.getDouble("dist_init", l1+l2),
                conf.getDouble("adj_init", 0));
        driver.setWaistAngle(conf.getDouble("waist_init", 0));
        wrist.setPosition(conf.getDouble("wrist_init", 0));
    }

    @Override
    public void stop() {
        base.close();
        extend.close();
        Logger.close();
    }

    private void moveTo(double[] armPos) {
        if (armPos == null) return;
        driver.moveTo(armPos[1], armPos[0]);
        if (armPos.length > 2) driver.setWaistAngle(armPos[2]);
        if (armPos.length > 3) extend.hold((int)armPos[3]);
    }

    private void moveToConfigured(String name, int terms) {
        String[] termNames = {"_adj", "_dist", "_waist", "_extend"};
        if (terms > termNames.length) throw new IllegalArgumentException(terms + "-term positions" +
                " are not currently supported!");
        double[] vals = new double[terms];
        for (int i = 0; i < terms; i++) {
            vals[i] = conf.getDouble(name + termNames[i], 0);
        }
        moveTo(vals);
    }

    private void setExtendedPositions() {
        moveTo(new double[] {conf.getDouble("raise_adj", 0), conf.getDouble("raise_dist", 0)});
    }

    @Override
    public void start() {
        start = System.currentTimeMillis();
        setExtendedPositions();
    }

    /**
     * Always use super.loop() if overriding
     */
    @Override
    public void loop() {
        buttonHelper_1.update();
        double newDist = -(gamepad1.right_stick_y * maxMove);
        double newAngle = (gamepad1.left_stick_y * maxMove);
        if (Math.abs(Utils.sum(rotateWindow)) > maxIncrease)
            newDist = 0;
        if (Math.abs(Utils.sum(extWindow)) > maxIncrease)
            newAngle = 0;
        addToEndOfRotateWindow(newDist);
        addToEndOfExtendWindow(newAngle);
        driver.moveTo(
                driver.getClawDistance() + newDist,
                driver.getArmAngle() + newAngle);
        if (gamepad2.dpad_left) {
            base.hold(base.getCurrentPosition() + base_speed);
        } else if (gamepad2.dpad_right) {
            base.hold(base.getCurrentPosition() - base_speed);
        } else {
            base.hold(base.getCurrentPosition());
        }

        wrist.setPosition(Utils.constrain(wrist.getPosition() + (wrist_speed * gamepad2.right_stick_y), 0, 1));

        driver.setWaistAngle(driver.getWaistAngle() - (gamepad1.left_stick_x * maxRotate));
        if (gamepad2.dpad_up) {
            extend.hold(extend.getCurrentPosition() + ext_speed);
        } else if (gamepad1.dpad_down) {
            extend.hold(extend.getCurrentPosition() - ext_speed);
        } else {
            extend.hold(extend.getCurrentPosition());
        }
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            return;
        }

        //Used to be A, but that would trigger the claw when Start+A was pressed to connect gamepad1
        if (buttonHelper_1.pressing(ButtonHelper.x)) {
            claw_closed = !claw_closed;
            if (claw_closed)
                claw.setPosition(clawCloseAmount);
            else
                claw.setPosition(clawOpenAmount);
        }

        if (buttonHelper_1.pressing(ButtonHelper.y)) {
            moveToConfigured("glyph", 3);
        }

        //We don't need this in PositionCollector
        if (buttonHelper_1.pressing(ButtonHelper.b) && !(this instanceof PositionFinder)) {
            moveToConfigured("cryptobox", 3);
        }

        if (buttonHelper_1.pressing(ButtonHelper.right_bumper)) {
            moveToConfigured("relic", 4);
        }

        if (gamepad1.left_bumper) {
            setInitialPositions();
        }

        telemetry.addData("Elapsed Time", Utils.elapsedTime(System.currentTimeMillis() - start));
        telemetry.addData("Claw", claw_closed ? "CLOSED" : "OPEN");
        if (!robot1) telemetry.addData("Limit Switch", (limit.getVoltage() < 0.8) ? "PRESSED" :
                "RELEASED");
        telemetry.addData("Arm Angle", Utils.shortFloat(driver.getArmAngle()));
        telemetry.addData("Distance", Utils.shortFloat(driver.getClawDistance()));
        telemetry.addData("Waist Position", Utils.shortFloat(driver.getWaistPos()));
        telemetry.addData("Waist Angle", Utils.shortFloat(driver.getWaistAngle()));
        telemetry.addData("Shoulder Position", Utils.shortFloat(driver.getShoulderPos()));
        telemetry.addData("Shoulder Angle", Utils.shortFloat(driver.getShoulderAngle()));
        telemetry.addData("Elbow Position", Utils.shortFloat(driver.getElbowPos()));
        telemetry.addData("Elbow Angle", Utils.shortFloat(driver.getElbowAngle()));
        telemetry.addData("Extend Position", extend.getCurrentPosition());
        telemetry.addData("Extend Minimum", extMin);
        telemetry.addData("Turntable Position", getTurntablePosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Running on", robot1 ? "Robot 1" : "Robot 2");
    }

    private void addToEndOfRotateWindow(double value) {
        rotateWindow[nextWindowSlotR++] = value;
        //If it overflows, start writing from the beginning again
        nextWindowSlotR %= rotateWindow.length;
    }
    private void addToEndOfExtendWindow(double value) {
        extWindow[nextWindowSlotE++] = value;
        //If it overflows, start writing from the beginning again
        nextWindowSlotE %= extWindow.length;
    }

}