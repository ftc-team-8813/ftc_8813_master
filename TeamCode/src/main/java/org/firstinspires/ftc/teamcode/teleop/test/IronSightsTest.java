package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.teleop.util.IronSightsJoystickControl;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Persistent;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="IronSights Test")
public class IronSightsTest extends OpMode {

    private IronSightsJoystickControl controller;
    private Logger log;

    @Override
    public void init() {
        try {
            Logger.init(new File(Config.storageDir + "logs/" + new SimpleDateFormat
                    ("yy_MM_dd_HH_mm_ss", Locale.US).format(new Date()) + ".log"));
        } catch (IOException e) { }
        log = new Logger("IronSights Test");
        Config conf = new Config(Config.configFile);
        Servo waist = hardwareMap.servo.get("s0");
        Servo shoulder = hardwareMap.servo.get("s1");
        Servo elbow = hardwareMap.servo.get("s2");
        Servo claw = hardwareMap.servo.get("s3");
        Servo wrist = hardwareMap.servo.get("s4");
        Servo yaw = hardwareMap.servo.get("s5");
        DcMotor base = hardwareMap.dcMotor.get("base");
        DcMotor extend = hardwareMap.dcMotor.get("extend");
        //Reverse motors if necessary
        if (conf.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (conf.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);
        //Load the IMU from autonomous data
        IMU imu = (IMU)Persistent.get("imu");
        if (imu == null) {
            //Create it if it isn't there
            imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
            imu.initialize(telemetry);
        }
        int quadrant;
        //Load the quadrant from autonomous data
        if (Persistent.get("quadrant") == null) {
            //Default to red upper for now
            quadrant = 4; //hehe
        } else {
            quadrant = (int)Persistent.get("quadrant");
        }
        imu.start();
        telemetry.clear();
        controller = new IronSightsJoystickControl(gamepad1, gamepad2,
                new Arm(conf, waist, shoulder, elbow, claw, wrist, yaw), conf, telemetry, base, extend, imu, quadrant, null, false, false);
    }

    @Override
    public void start() {
        controller.start();
    }

    @Override
    public void loop() {
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            return;
        }

        controller.loop();
    }

    @Override
    public void stop() {
        controller.stop();
        Persistent.clear();
        Logger.close();
    }
}
