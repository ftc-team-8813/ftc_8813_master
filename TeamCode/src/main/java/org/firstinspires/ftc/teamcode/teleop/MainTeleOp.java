package org.firstinspires.ftc.teamcode.teleop;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

/**
 * Very small test TeleOp program
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    private Robot robot;
    private int intake_mode;
    private boolean slow;
    private ButtonHelper buttonHelper_1;
    private ButtonHelper buttonHelper_2;
    
    @Override
    public void init()
    {
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        buttonHelper_1 = new ButtonHelper(gamepad1);
        buttonHelper_2 = new ButtonHelper(gamepad2);
        slow = false;
        robot.intakeFlipper.setPosition(0); // Initialize to 0
    }
    
    @Override
    public void loop()
    {
        double mult = 0.75;
        if (slow) mult = 0.375;
        robot.leftFront.setPower(-gamepad1.left_stick_y * mult);
        robot.leftRear.setPower(-gamepad1.left_stick_y * mult);
        robot.rightFront.setPower(-gamepad1.right_stick_y * mult);
        robot.rightRear.setPower(-gamepad1.right_stick_y * mult);

        robot.leftDunk.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.rightDunk.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        robot.intake.setPower(intake_mode * 0.5);
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

        if (buttonHelper_1.pressing(ButtonHelper.dpad_up))
        {
            robot.intakeFlipper.setPosition(0);
        }
        else if (buttonHelper_1.pressing(ButtonHelper.dpad_left))
        {
            robot.intakeFlipper.setPosition(0.115);
        }
        else if (buttonHelper_1.pressing(ButtonHelper.dpad_right))
        {
            robot.intakeFlipper.setPosition(0.141);
        }
        else if (buttonHelper_1.pressing(ButtonHelper.dpad_down))
        {
            robot.intakeFlipper.setPosition(0.170);
        }

        if (buttonHelper_1.pressing(ButtonHelper.right_stick_button))
        {
            slow = !slow;
        }
        telemetry.addData("Intake Mode", intake_mode);
        telemetry.addData("Slow", slow);
    }
}
