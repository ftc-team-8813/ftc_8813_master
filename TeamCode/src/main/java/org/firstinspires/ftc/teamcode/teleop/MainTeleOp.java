package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;

/**
 * Very small test TeleOp program
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    
    private DcMotor left, right;
    private DcMotor flipper, intake;
    private MotorController mc;
    private double offset;
    private ButtonHelper buttonHelper_2;
    private int intake_mode = 0;
    
    
    @Override
    public void init()
    {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        flipper = hardwareMap.dcMotor.get("flipper");
        intake = hardwareMap.dcMotor.get("intake");
        mc = new MotorController(flipper, new Config("config.properties"));
        buttonHelper_2 = new ButtonHelper(gamepad2);
    }
    
    @Override
    public void loop()
    {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);
        mc.hold(50 + (int)offset);
        flipper.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        intake.setPower(intake_mode);
        offset += (gamepad2.right_trigger - gamepad2.left_trigger) * 10;
        if (buttonHelper_2.pressing(ButtonHelper.right_bumper))
        {
            if (intake_mode == 1) intake_mode = 0;
            else intake_mode = 1;
        }
        else if (buttonHelper_2.pressing(ButtonHelper.left_bumper))
        {
            if (intake_mode == -1) intake_mode = 0;
            else intake_mode = -1;
        }
        telemetry.addData("Flipper position", flipper.getCurrentPosition());
        telemetry.addData("Flipper target", mc.getTargetPosition());
        telemetry.addData("Intake Mode", intake_mode);
    }
}
