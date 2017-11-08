package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.teleop.util.ServoAngleFinder;

/**
 * Created by aidan on 11/3/17.
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode {

    private ArmDriver driver;
    private Servo claw;
    private DcMotor base, extend;
    private DigitalChannel limit;
    private Integer extMin = null;
    private int extRange;
    private Config conf = new Config(Config.configFile);
    private double maxMove = conf.getDouble("max_move", 0.02);
    private double maxRotate = conf.getDouble("max_rotate_speed", 0.1);
    private boolean aHeld;
    private boolean claw_closed;
    private double clawCloseAmount = conf.getDouble("claw_closed", 0);
    private double clawOpenAmount = conf.getDouble("claw_open", 0);

    @Override
    public void init() {
        Servo waist = hardwareMap.servo.get("s0");
        Servo shoulder = hardwareMap.servo.get("s1");
        Servo elbow = hardwareMap.servo.get("s2");
        claw = hardwareMap.servo.get("s3");
        base = hardwareMap.dcMotor.get("base");
        extend = hardwareMap.dcMotor.get("extend");
        limit = hardwareMap.digitalChannel.get("limit");
        driver = new ArmDriver(waist, shoulder, elbow);

        extRange = conf.getInt("ext_range", 0);

        setInitialPositions();
        ServoAngleFinder.create(hardwareMap);
    }

    private void setInitialPositions() {
        driver.moveTo(conf.getDouble("waist_init", 0),
                      conf.getDouble("shoulder_init", 0),
                      conf.getDouble("elbow_init", 0));
    }

    @Override
    public void loop() {
        driver.setArmX(driver.getClawX()-(gamepad1.left_stick_y * maxMove));
        driver.setArmY(driver.getClawY()-(gamepad1.right_stick_y * maxMove));
        base.setPower(gamepad1.left_stick_x * 0.5);
        driver.setWaistAngle(driver.getWaistAngle()+(gamepad1.right_stick_x * maxRotate));
        //getState same as isPressed, except for DigitalChannels (which are needed for REV sensors)
        if (!limit.getState()) {
            //Only allows user to go forward if the minimum switch has been triggered.
            if (gamepad1.dpad_up && extMin != null && extend.getCurrentPosition() < extMin + extRange) {
                extend.setPower(1);
            } else if (gamepad1.dpad_down) {
                extend.setPower(-1);
            } else {
                extend.setPower(0);
            }
        } else {
            extend.setPower(0);
            extMin = extend.getCurrentPosition();
        }
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {}

        //Used to be A, but that would trigger the claw when Start+A was pressed to connect gamepad1
        if (gamepad1.x) {
            if (!aHeld) {
                aHeld = true;
                claw_closed = !claw_closed;
                if (claw_closed)
                    claw.setPosition(clawCloseAmount);
                else
                    claw.setPosition(clawOpenAmount);
            }
        } else {
            aHeld = false;
        }

        if (gamepad1.left_bumper) {
            setInitialPositions();
        }
    }
}
