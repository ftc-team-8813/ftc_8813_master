package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

/**
 * Very small test TeleOp program
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    
    private DcMotor left, right;
    private DcMotor lifter, intake;
    private Servo intakeFlipper;
    private ButtonHelper buttonHelper_2;
    private int intake_mode = 0;
    
    
    @Override
    public void init()
    {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        lifter = hardwareMap.dcMotor.get("lifter");
        intake = hardwareMap.dcMotor.get("intake");
        intakeFlipper = hardwareMap.servo.get("flipper");
        buttonHelper_2 = new ButtonHelper(gamepad2);
        intakeFlipper.setPosition(0.17);
    }
    
    @Override
    public void loop()
    {
        left.setPower(gamepad1.right_stick_y * 0.75);
        right.setPower(-gamepad1.left_stick_y * 0.75);
        lifter.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        intake.setPower(intake_mode * 0.5);
        if (gamepad2.right_bumper)
        {
            intake_mode = 1;
        }
        else if (gamepad2.left_bumper)
        {
            intake_mode = -1;
        }
        else
        {
            intake_mode = 0;
        }
        if (buttonHelper_2.pressing(ButtonHelper.dpad_up))
        {
            intakeFlipper.setPosition(0);
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_down))
        {
            intakeFlipper.setPosition(0.170);
        }
        telemetry.addData("Intake Mode", intake_mode);
    }
}
