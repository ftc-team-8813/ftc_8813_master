package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.util.IMUMotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.QuadrantChooser;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.util.IronSightsJoystickControl;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.teleop.util.ServoAngleFinder;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Persistent;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Main TeleOp control to control the {@link ArmDriver}. Most disorganized part of the code; can
 * be cleaned up
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode {

    protected volatile int quadrant;
    protected IronSightsJoystickControl controller;
    protected Logger log;
    private IMU imu;
    private Thread chooser;
    private boolean waiting;
    private Config conf;
    private Servo colorArm;

    @Override
    public void init() {

        //Initialize logging
        try {
            Logger.init(new File(Config.storageDir + "logs/" + new SimpleDateFormat
                    ("yy_MM_dd_HH_mm_ss", Locale.US).format(new Date()) + ".log"));
        } catch (IOException e) { }

        log = new Logger("TeleOp");

        //Initialize config
        //If we're on robot 1, use robot 1 config
        final Config conf;
        if (hardwareMap.getAll(LynxModule.class).get(0).getSerialNumber().toString().equals
                ("DQ168FFD")) {
            throw new IllegalStateException("This TeleOp cannot be run on robot 1! Please use the old program");
        } else {
            conf = new Config("config.properties");
        }
        this.conf = conf;

        log = new Logger("IronSights Test");
        final Servo waist = hardwareMap.servo.get("s0");
        final Servo shoulder = hardwareMap.servo.get("s1");
        final Servo elbow = hardwareMap.servo.get("s2");
        final Servo claw = hardwareMap.servo.get("s3");
        final Servo wrist = hardwareMap.servo.get("s4");
        final Servo yaw = hardwareMap.servo.get("s5");
        colorArm = hardwareMap.servo.get("s6");
        final DcMotor base = hardwareMap.dcMotor.get("base");
        final DcMotor extend = hardwareMap.dcMotor.get("extend");
        //Reverse motors if necessary
        if (conf.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (conf.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);
        //Load the IMU from autonomous data
        imu = (IMU)Persistent.get("imu");
        if (imu == null) {
            //Create it if it isn't there
            imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
            imu.initialize(telemetry);
        }
        final int[] quadrant = new int[1];
        //Load the quadrant from autonomous data
        if (Persistent.get("quadrant") == null) {
            //Default to red upper for now
            quadrant[0] = 4; //hehe
        } else {
            quadrant[0] = (int)Persistent.get("quadrant");
        }
        final TaskClassifyPictograph.Result findResult =
                (TaskClassifyPictograph.Result)Persistent.get("findResult");
        imu.start();
        telemetry.clear();
        if (Persistent.get("quadrant") == null) {
            quadrant[0] = -1;
            chooser = new Thread(new Runnable() {
                @Override
                public void run() {
                    QuadrantChooser ch = new QuadrantChooser(telemetry);
                    quadrant[0] = ch.chooseQuadrant();
                    if (quadrant[0] < 0) return;
                    controller = new IronSightsJoystickControl(gamepad1, gamepad2,
                            new Arm(conf, waist, shoulder, elbow, claw, wrist, yaw), conf,
                            telemetry, base, extend, imu, quadrant[0], findResult, MainTeleOp.this instanceof PositionFinder);
                }
            });
            chooser.setDaemon(true);
            chooser.start();
        } else {
            quadrant[0] = (int)Persistent.get("quadrant");
            controller = new IronSightsJoystickControl(gamepad1, gamepad2,
                    new Arm(conf, waist, shoulder, elbow, claw, wrist, yaw), conf,
                    telemetry, base, extend, imu, quadrant[0], findResult, this instanceof  PositionFinder);
        }

        //Set up
    }



    @Override
    public void stop() {
        imu.stop();
        if (chooser != null) chooser.interrupt();
        if (controller != null) controller.stop();
        Persistent.clear();
        Logger.close();
    }

    @Override
    public void start() {
        if (conf.getDoubleArray("color_arm_positions") != null) colorArm.setPosition(conf
                .getDoubleArray("color_arm_positions")[1]);
        if (controller != null) controller.start();
    }

    /**
     * Override this method instead of loop(). Does nothing by default; super.run() not required.
     */
    public void run() {}

    @Override
    public final void loop() {
        if (controller == null) {
            //Please choose a quadrant
            waiting = true;
        } else {
            if (waiting) {
                waiting = false;
                controller.start();
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return;
            }
            run();
            controller.loop();
        }
    }

}