package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Joseph Murphy on 9/20/2017.
 */
@TeleOp(name = "TeleOpTest")
public class TaskSomethingIDontKnow extends OpMode {
    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
    }

    @Override
    public void loop() {
        double right_y = -gamepad1.right_stick_y;
        motor.setPower(right_y);
        if(gamepad1.right_bumper | gamepad1.left_bumper){
            right_y = -.5 * gamepad1.right_stick_y;
            motor.setPower(right_y);
        }
    }
}
