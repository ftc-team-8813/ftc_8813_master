package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Very small test TeleOp program
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    
    private DcMotor left, right;
    private DcMotor flipper, intake;
    
    @Override
    public void init()
    {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        flipper = hardwareMap.dcMotor.get("flipper");
        intake = hardwareMap.dcMotor.get("intake");
    }
    
    @Override
    public void loop()
    {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);
        flipper.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        intake.setPower((gamepad1.right_bumper ? 1 : 0) + (gamepad1.left_bumper ? -1 : 0));
    }
}
