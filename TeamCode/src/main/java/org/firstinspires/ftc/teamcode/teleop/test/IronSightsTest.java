package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.teleop.util.IronSightsArmDriver;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="IronSights Test")
public class IronSightsTest extends OpMode {

    private IronSightsArmDriver driver;
    private Logger log;
    private double i = 5, j = 5, wrist = Math.PI;

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
        driver = new IronSightsArmDriver(new Arm(conf, waist, shoulder, elbow, claw, wrist, yaw),
                null, null, conf);
        //double[] init = conf.getDoubleArray("init");
        //Initialize the servos and set an initial position so that the angles are not zero
        // because that makes the Newton-Raphson equation return NaN since there are infinite
        // solutions!!
        driver.driveManual(0, 0, 0,Math.PI/4, Math.PI/2, 5*Math.PI/4);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            return;
        }

        double x_inc = gamepad1.left_stick_x;
        double y_inc = -gamepad1.left_stick_y;
        double wrist_inc = gamepad1.right_stick_y;

        if (Math.abs(x_inc) > 0.001 || Math.abs(y_inc) > 0.001) {
            i += x_inc;
            j += y_inc;
            wrist += wrist_inc;
            if (wrist > 3.0*Math.PI/2.0) {
                wrist = 3.0*Math.PI/2.0;
            } else if (wrist < Math.PI/2.0) {
                wrist = Math.PI/2;
            }
            driver.moveArmTo(i, j, wrist);
        }

        if (gamepad1.left_bumper) {
            i = 5;
            j = 5;
            wrist = Math.PI;
        }
    }

    @Override
    public void stop() {
        Logger.close();
    }
}
