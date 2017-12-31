package org.firstinspires.ftc.teamcode.teleop.test;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

/**
 * Created by aidan on 12/21/17.
 */

@TeleOp(name = "PID Tuner", group = "test")
public class PIDTuner extends OpMode {

    private MotorController controller;
    private ButtonHelper buttons;
    private int changing = 0;

    @Override
    public void init() {
        controller = new MotorController(hardwareMap.dcMotor.get("motor"));
        buttons = new ButtonHelper(gamepad1);
        controller.hold(0);
    }

    @Override
    public void loop() {
        controller.hold(controller.getTargetPosition() - (int)(gamepad1.left_stick_y*50));
        if (buttons.pressing(ButtonHelper.dpad_up)) {
            changing++;
            changing %= 3;
        }
        if (buttons.pressing(ButtonHelper.dpad_down)) {
            changing--;
            if (changing < 0) changing = 2;
        }
        double[] constants = controller.getPIDConstants();
        switch (changing) {
            case 0:
                telemetry.addData("Changing", "Proportional Gain");
                controller.setPIDConstants(constants[0] - gamepad1.right_stick_y / 100, constants[1], constants[2]);
                break;
            case 1:
                telemetry.addData("Changing", "Integral Gain");
                controller.setPIDConstants(constants[0], constants[1] - gamepad1.right_stick_y / 100, constants[2]);
                break;
            case 2:
                telemetry.addData("Changing", "Derivative Gain");
                controller.setPIDConstants(constants[0], constants[1], constants[2] - gamepad1.right_stick_y / 100);
                break;
        }
        telemetry.addData("Target", controller.getTargetPosition());
        telemetry.addData("Position", controller.getCurrentPosition());
        telemetry.addData("kP", constants[0]);
        telemetry.addData("kI", constants[1]);
        telemetry.addData("kD", constants[2]);
    }

    @Override
    public void stop() {
        controller.close();
    }
}
