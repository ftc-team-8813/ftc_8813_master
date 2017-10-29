package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.Config;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "Tele-OP Test Controller")
public class TestOPMode extends OpMode {
    double servopos;
    Servo servo;
    Servo servo2;
    Servo servo3;
    private List<Double> list = new ArrayList<>();
    private List<Double> list2 = new ArrayList<>();
    private List<Double> list3 = new ArrayList<>();

    @Override
    public void init() {
        servo = hardwareMap.servo.get("s0");
        servo2 = hardwareMap.servo.get("s1");
        servo3 = hardwareMap.servo.get("s2");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0) {
            servopos = servopos - .01;
            servo.setPosition(servopos);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
        if (gamepad1.left_stick_y < 0) {
            servopos = servopos + .01;
            servo.setPosition(servopos);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
        if (servopos < 0)
            servopos = 0;
        if (servopos > 1)
            servopos = 1;

        if (gamepad1.a) {
            list.add(servopos);
            while (gamepad1.a) ;
        }
        if (gamepad1.b) {
            for (double d : list) {
                servo.setPosition(d);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    return;
                }
            }
            while (gamepad1.b) ;
        }
        telemetry.addData("ServoPosition", servopos);
    }

    public void write() {
        try {
            DataOutputStream out = new DataOutputStream(new FileOutputStream(Config.storageDir + "/ServoPos.bin"));
            

        }   catch (IOException e) {
            e.printStackTrace();
        }
    }
}