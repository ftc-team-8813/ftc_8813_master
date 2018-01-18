package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.DcMotorUtil;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.teleop.util.ServoAngleFinder;
import org.firstinspires.ftc.teamcode.util.Utils;

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
    protected DcMotor base, extend;
    //Limit switch
//    protected DigitalChannel limit;
    protected AnalogInput limit;
    //Extend motor minimum position
    protected Integer extMin = null;
    //Range of extend motor
    protected int extRange;
    //Configuration
    protected Config conf = new Config(Config.configFile);

    protected ButtonHelper buttonHelper_1;
    //Maximum speed of arm servos (some arbitrary unit)
    private double maxMove = conf.getDouble("max_move", 0.002);
    //Maximum amount of change allowed in 200ms second
    private double maxIncrease = conf.getDouble("max_inc", 0.02);
    //Maximum speed of waist servo (radians per 20ms)
    private double maxRotate = conf.getDouble("max_rotate_speed", 0.02);

    private double[] wristRange = conf.getDoubleArray("wrist_range");
    private double wrist_speed = conf.getDouble("wrist_speed", 0.01);

    private double[] rotateWindow = new double[10];
    private double[] extWindow = new double[10];
    private int nextWindowSlotR = 0;
    private int nextWindowSlotE = 0;
    //Whether the claw open/close button is currently being held down
    private boolean aHeld;
    //Whether the claw is open or closed
    private boolean claw_closed;
    //The amount to close the claw
    private double clawCloseAmount = conf.getDouble("claw_closed", 0);
    //The amount to open the claw
    private double clawOpenAmount = conf.getDouble("claw_open", 0);

    private double l1 = conf.getDouble("l1", 1);
    private double l2 = conf.getDouble("l2", 1);

    private long start = 0;

    @Override
    public void init() {
        buttonHelper_1 = new ButtonHelper(gamepad1);
        //Get motors and servos from hardware map
        Servo waist = hardwareMap.servo.get("s0");
        Servo shoulder = hardwareMap.servo.get("s1");
        Servo elbow = hardwareMap.servo.get("s2");
        wrist = hardwareMap.servo.get("s4");
        claw = hardwareMap.servo.get("s3");
        base = hardwareMap.dcMotor.get("base");
        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend = hardwareMap.dcMotor.get("extend");
        //extend has no encoder
        limit = hardwareMap.analogInput.get("limit");
        //Initialize arm controller
        driver = new ArmDriver(waist, shoulder, elbow, l1, l2, conf);

        //Get extend motor range
        extRange = conf.getInt("ext_range", Integer.MAX_VALUE/2);

        if (conf.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (conf.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);

        if (wristRange != null) {
            wrist.scaleRange(wristRange[0], wristRange[1]);
        }

        //Set up
        setInitialPositions();
        ServoAngleFinder.create(hardwareMap);
    }

    private void setInitialPositions() {
        claw.setPosition(conf.getDouble("claw_closed", 0));
        claw_closed = true;
        driver.moveTo(conf.getDouble("dist_init", l1+l2),
                      conf.getDouble("adj_init", 0));
        driver.setWaistAngle(conf.getDouble("waist_init", 0));
        wrist.setPosition(conf.getDouble("wrist_init", 0));
    }

    private void setExtendedPositions() {
        driver.moveTo(conf.getDouble("dist_out", l1+l2),
                      conf.getDouble("adj_out", 0));
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
            base.setPower(0.8);
        } else if (gamepad2.dpad_right) {
            base.setPower(-0.8);
        } else {
            base.setPower(0);
        }

        if (gamepad1.dpad_up) {
            wrist.setPosition(Utils.constrain(wrist.getPosition() + wrist_speed, 0, 1));
        } else if (gamepad1.dpad_down) {
            wrist.setPosition(Utils.constrain(wrist.getPosition() - wrist_speed, 0, 1));
        }

        driver.setWaistAngle(driver.getWaistAngle() - (gamepad1.left_stick_x * maxRotate));
        //getState same as !isPressed, except for DigitalChannels (which are needed for REV sensors)
        //For AnalogInputs, getVoltage() should be around 0 when active
        //CMOS logic LOW is < 0.8V
        //if (limit.getVoltage() >= 0.8) {
        //Only allows user to go backward if the minimum switch hasn't been triggered.
        if (gamepad2.dpad_down) {
            extend.setPower(-1);
        } else {
            extend.setPower(0);
        }
        //} else {
        //    extend.setPower(0);
        //    extMin = extend.getCurrentPosition();
        //}
        if (gamepad2.dpad_up /*&& extMin != null && extend.getCurrentPosition() < extMin + extRange*/) {
            extend.setPower(1);
        } else if (extend.getPower() != -1) {
            extend.setPower(0);
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
            driver.moveTo(
                    conf.getDouble("dist_glyphs", l1+l2),
                    conf.getDouble("adj_glyphs", 0));
            driver.setWaistAngle(conf.getDouble("waist_glyphs", 0));
            DcMotorUtil.holdUntilComplete(base, conf.getInt("base_glyphs", 0), 1);
        }

        if (buttonHelper_1.pressing(ButtonHelper.b)) {
            driver.moveTo(
                    conf.getDouble("dist_cryptobox", l1+l2),
                    conf.getDouble("adj_cryptobox", 0));
            driver.setWaistAngle(conf.getDouble("waist_cryptobox", 0));
            DcMotorUtil.holdUntilComplete(base, conf.getInt("base_cryptobox", 0), 1);
        }

        if (gamepad1.left_bumper) {
            setInitialPositions();
        }

        telemetry.addData("Elapsed Time", Utils.elapsedTime(System.currentTimeMillis() - start));
        telemetry.addData("Claw", claw_closed ? "CLOSED" : "OPEN");
        telemetry.addData("Limit Switch", (limit.getVoltage() < 0.8) ? "PRESSED" : "RELEASED");
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
        telemetry.addData("Turntable Position", base.getCurrentPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
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
