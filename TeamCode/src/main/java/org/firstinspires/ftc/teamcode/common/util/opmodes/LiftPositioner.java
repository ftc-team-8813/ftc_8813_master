package org.firstinspires.ftc.teamcode.common.util.opmodes;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "LiftPositioner")
public class LiftPositioner extends OpMode {
    DataStorage text2 = new DataStorage(new File(Config.storageDir + "liftencoderpos.txt"));
    DcMotor lift;

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotor.class, "slide lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("", "Using Left Joystick Y-Axis, Move Lift to Highest Position");
        telemetry.addData("", "Press B to Save Highest Position");
        if (gamepad1.b){
            text2.addNumber("Highest Position", lift.getCurrentPosition());
        }

        if (gamepad1.left_stick_y != 0){
            lift.setPower(gamepad1.left_stick_y);
        }

    }

    @Override
    public void stop(){
        text2.save();
    }
}
