package org.firstinspires.ftc.teamcode.teleop;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Scheduler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.io.IOException;

/**
 * Main driver control
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    private Robot robot;
    private int intake_mode;
    private boolean slow;
    private ButtonHelper buttonHelper_1;
    private ButtonHelper buttonHelper_2;

    private boolean liftingDunk = false;
    private boolean droppingDunk = false;
    private boolean dunkDown = false;
    private boolean pivotingDown = false;
    private int limit_press_count = 0;

    private static final double dunk_nearly_down = 0.57;

    private static final int intake_out = 300;

    private Scheduler scheduler = new Scheduler();
    private Logger log;

    private long start;

    
    @Override
    public void init()
    {
        try { Logger.init(); } catch (IOException e) { throw new RuntimeException(e); }
        log = new Logger("Driver Control");
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        buttonHelper_1 = new ButtonHelper(gamepad1);
        buttonHelper_2 = new ButtonHelper(gamepad2);
        slow = false;
        robot.pivot.startLogging();
        try
        {
            robot.initPivot();
        }  catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        robot.dunk.setPosition(Robot.dunk_min);
    }

    @Override
    public void start()
    {
        start = System.currentTimeMillis();
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
        if (robot.liftLimitDown.pressed() && !dunkDown)
        {
            droppingDunk = true;
            liftingDunk = false;
            dunkDown = true;
        }
        if (robot.liftLimitDown.pressed())
        {
            if (liftPower > 0)
            {
                liftingDunk = true;
                droppingDunk = false;
                dunkDown = false;
                if (robot.pivot.getCurrentPosition() < intake_out) robot.pivot.hold(intake_out);
            }
            if (liftPower >= 0)
            {
                robot.dunkLift.setPower(liftPower);
            }
        }
        else if (robot.liftLimitUp.pressed())
        {
            if (liftPower <= 0)
            {
                robot.dunkLift.setPower(liftPower);
            }
            if (liftPower < 0)
            {
                if (robot.pivot.getCurrentPosition() < intake_out) robot.pivot.hold(intake_out);
            }
        }
        else
        {
            robot.dunkLift.setPower(liftPower);
            if (liftPower != 0 && robot.pivot.getCurrentPosition() < intake_out) robot.pivot.hold(intake_out);
        }

        if (robot.pullupLimit.pressed()) limit_press_count++;
        else limit_press_count = 0;

        if (gamepad2.left_bumper && (!robot.pullupLimit.pressed() || limit_press_count < 50))
        {
            robot.pullUp.setPower(1);
            if (robot.pivot.getCurrentPosition() < intake_out) robot.pivot.hold(intake_out);
        }
        else if (gamepad2.right_bumper)
        {
            robot.pullUp.setPower(-1);
            if (robot.pivot.getCurrentPosition() < intake_out) robot.pivot.hold(intake_out);
        }
        else
        {
            robot.pullUp.setPower(0);
        }

        if (buttonHelper_1.pressing(ButtonHelper.b))
        {
            liftingDunk = false;
            droppingDunk = false;
            if (robot.dunk.getPosition() < Robot.dunk_up - 0.1)
                robot.dunk.setPosition(Robot.dunk_up);
            else
            {
                robot.dunk.setPosition(Robot.dunk_dunk);
                scheduler.add("Pull Back Dunk",750, new Runnable()
                {
                    @Override
                    public void run()
                    {
                        robot.dunk.setPosition(Robot.dunk_up);
                    }
                });
            }
        }

        if (liftingDunk)
        {
            if (robot.dunk.getPosition() > Robot.dunk_up) robot.dunk.setPosition(robot.dunk.getPosition() - 0.03);
            else liftingDunk = false;
//            robot.dunk.setPosition(Robot.dunk_up);
//            liftingDunk = false;
        }
        else if (droppingDunk)
        {
            if (robot.dunk.getPosition() < dunk_nearly_down) robot.dunk.setPosition(dunk_nearly_down);
            else if (robot.dunk.getPosition() < Robot.dunk_min) robot.dunk.setPosition(robot.dunk.getPosition() + 0.05);
            else droppingDunk = false;
        }

        if (buttonHelper_2.pressing(ButtonHelper.x))
        {
            if (robot.hook.getPosition() < Robot.HOOK_CLOSED)
                robot.hook.setPosition(Robot.HOOK_CLOSED);
            else
                robot.hook.setPosition(Robot.HOOK_OPEN);
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
            robot.pivot.hold(800);
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_right))
        {
            robot.pivot.stopHolding();
            pivotingDown = true;
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_down))
        {
            robot.pivot.hold(1300);
        }
        if (pivotingDown)
        {
            if (robot.pivot.isHolding()) pivotingDown = false;
            if (robot.intakePivot.getCurrentPosition() >= 400) pivotingDown = false;
            if (!pivotingDown) robot.intakePivot.setPower(0);
            else robot.intakePivot.setPower(0.75);
        }

        if (buttonHelper_1.pressing(ButtonHelper.right_stick_button))
        {
            slow = !slow;
        }

        scheduler.update();
        telemetry.addData("Time", Utils.elapsedTime(System.currentTimeMillis() - start));
        telemetry.addData("Intake Mode", intake_mode);
        telemetry.addData("Slow", slow);
        telemetry.addData("Pivot limit switch", robot.pivotLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Lower limit switch", robot.liftLimitDown.pressed() ? "Pressed" : "Released");
        telemetry.addData("Upper limit switch", robot.liftLimitUp.pressed() ? "Pressed" : "Released");
        telemetry.addData("Pullup limit switch", robot.pullupLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Dunk Position", robot.dunk.getPosition());
    }

    @Override
    public void stop()
    {
        robot.uninitialize();
    }
}
