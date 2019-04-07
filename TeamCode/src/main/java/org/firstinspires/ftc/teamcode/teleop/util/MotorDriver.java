package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Driver")
public class MotorDriver extends OpMode
{

    private DcMotor motor;

    @Override
    public void init()
    {
        motor = hardwareMap.dcMotor.get("test motor");
    }

    @Override
    public void loop()
    {
        if (gamepad1.dpad_up) motor.setPower(0.5);
        else if (gamepad1.dpad_down) motor.setPower(-0.5);
        else motor.setPower(0);
    }
}
