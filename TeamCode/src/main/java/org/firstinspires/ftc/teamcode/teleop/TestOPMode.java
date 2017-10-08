package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by anna1 on 10/4/2017.
 */
@TeleOp (name = "Testing My First Program!!!")
public class TestOPMode extends OpMode {
    double servopos;
    Servo servo;
    private List<Double> list = new ArrayList<>();
    @Override
    public void init() {
        servo = hardwareMap.servo.get("s0");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0){
            servopos = servopos - .01;
            servo.setPosition(servopos);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {}
        }
        if (gamepad1.left_stick_y < 0){
            servopos = servopos + .01;
            servo.setPosition(servopos);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {}
        }
        if (servopos < 0)
            servopos = 0;
        if (servopos > 1)
            servopos = 1;

        if (gamepad1.a) {
            list.add(servopos);
            while (gamepad1.a);
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
            while (gamepad1.b);
        }
        telemetry.addData("ServoPosition", servopos);
    }
}
