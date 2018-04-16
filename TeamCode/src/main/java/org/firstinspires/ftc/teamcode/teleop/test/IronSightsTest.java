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
import static java.lang.Math.*;

@TeleOp(name="IronSights Test")
public class IronSightsTest extends OpMode {

    private IronSightsArmDriver driver;
    private Logger log;
    private double i, j, k, wrist;

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
        driver.driveManual(0, 0, 0,Math.PI/4, Math.PI/2, Math.PI/2);
        for (int i = 0; i < 6; i++) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                return;
            }
            driver.driveManual(0, 0, 0, Math.PI / 4, Math.PI / 2, PI/2 + i*toDegrees(15));
            telemetry.addData("Driving to ", i*15 + 90);
            telemetry.update();
        }
        i = driver.getX();
        j = driver.getY();


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

        double x_inc = gamepad1.right_stick_x;
        double y_inc = -gamepad1.right_stick_y;
        double z_inc = -gamepad1.left_stick_y;
        double wrist_inc = (gamepad1.right_trigger - (gamepad1.right_bumper ? 1 : 0)) / 10;

        if (Math.abs(x_inc) > 0.001 || Math.abs(y_inc) > 0.001 || Math.abs(wrist_inc) > 0.001 ||
        Math.abs(z_inc) > 0.001) {
            i += x_inc;
            j += y_inc;
            k += z_inc;
            wrist += wrist_inc;
            if (wrist > 3*PI/2) {
                wrist = 3*PI/2;
            } else if (wrist < PI/2) {
                wrist = PI/2;
            }
            driver.moveArmTo(i, j, k, wrist);

            if (gamepad1.left_bumper) {
                i = 5;
                j = 5;
                wrist = PI;
            }
        }

        telemetry.addData("i", i);
        telemetry.addData("j", j);
        telemetry.addData("k", k);
        telemetry.addData("tw", wrist);
        telemetry.addData("Waist", driver.getWaistAngle());
        telemetry.addData("Shoulder", driver.getShoulderAngle());
        telemetry.addData("Elbow", driver.getElbowAngle());
        telemetry.addData("Wrist", driver.getWristAngle());

    }

    @Override
    public void stop() {
        Logger.close();
    }
}
