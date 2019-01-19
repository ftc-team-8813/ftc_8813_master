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
        try
        {
            robot.initPivot();
        }  catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        robot.dunk.setPosition(0);
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

        double liftPower = -(gamepad2.right_trigger - gamepad2.left_trigger);
        if (liftPower >= 0 || !robot.liftLimit.pressed())
        {
            robot.leftDunk.setPower(-liftPower);
            robot.rightDunk.setPower(liftPower);
            if (Math.abs(liftPower) > 0) robot.dunk.setPosition(0.028);
        }
        if (liftPower <= 0 && robot.liftLimit.pressed())
            robot.dunk.setPosition(0);

        if (buttonHelper_1.pressing(ButtonHelper.b))
        {
            if (robot.dunk.getPosition() == 0.2)
                robot.dunk.setPosition(0);
            else
                robot.dunk.setPosition(0.2);
        }

        robot.intake.setPower(intake_mode * 0.5);
        if (gamepad1.right_bumper)
        {
            intake_mode = 1;
        }
        else if (gamepad1.left_bumper)
        {
            intake_mode = -1;
        }
        else
        {
            intake_mode = 0;
        }

        if (buttonHelper_2.pressing(ButtonHelper.dpad_up))
        {
            robot.pivot.hold(15);
            // TODO run rollers when at target
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_left))
        {
            robot.pivot.hold(600);
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_right))
        {
            robot.pivot.hold(988);
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_down))
        {
            robot.pivot.hold(1300);
        }

        if (buttonHelper_1.pressing(ButtonHelper.right_stick_button))
        {
            slow = !slow;
        }
        telemetry.addData("Intake Mode", intake_mode);
        telemetry.addData("Slow", slow);
        telemetry.addData("Pivot limit switch", robot.pivotLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Lift limit switch", robot.liftLimit.pressed() ? "Pressed" : "Released");
    }

    @Override
    public void stop()
    {
        robot.uninitialize();
    }
}
