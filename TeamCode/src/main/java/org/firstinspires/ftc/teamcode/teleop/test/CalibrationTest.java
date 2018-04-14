package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.util.IronSightsArmDriver;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="Calibration Test")
public class CalibrationTest extends OpMode {

    private Logger log;
    private IronSightsArmDriver driver;
    private double t;

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
    }

    @Override
    public void loop() {
        driver.driveManual(0, 0, 0, Math.PI/4, Math.PI/2, Math.PI/2 + t);
        t -= gamepad1.left_stick_y / 10;
        telemetry.addData("Angle: ", Math.toDegrees(Math.PI/2 + t));
        telemetry.addData("Position: ", driver.getWristPosition());
    }
}
