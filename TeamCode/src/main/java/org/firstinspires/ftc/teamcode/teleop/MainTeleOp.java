package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Scheduler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.VMStats;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.io.IOException;

/**
 * Main driver control
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode
{
    private Robot robot;
    private boolean slow;
    private ButtonHelper buttonHelper_1;
    private ButtonHelper buttonHelper_2;

    private boolean liftingDunk = false;
    private boolean droppingDunk = false;
    private boolean dunkDown = false;
    private boolean liftDown = true;
    private int limit_press_count = 0;

    private double dunk_nearly_down;

    private Scheduler scheduler = new Scheduler();
    private Logger log;

    private long start;

    private int fps = 0;
    private int framecount;
    private long lastFrame = 0;

    private double prevLeftJoy, prevRightJoy;

    private VMStats cpustat;

    
    @Override
    public void init()
    {
        try { Logger.init(); } catch (IOException e) { throw new RuntimeException(e); }
        log = new Logger("Driver Control");
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        dunk_nearly_down = robot.dunk_up;
        buttonHelper_1 = new ButtonHelper(gamepad1);
        buttonHelper_2 = new ButtonHelper(gamepad2);
        slow = false;
        // robot.pivot.startLogging();
        robot.intakePivot.setPosition(robot.pivot_up);
        robot.dunk.setPosition(robot.dunk_min);
        robot.calibrateAll();
        cpustat = new VMStats(1);
    }

    @Override
    public void init_loop()
    {
        if (robot.working()) telemetry.addData("Calibrating intake pivot", "please wait");
        else
        {
            telemetry.addData("Calibration", "Finished");
            robot.intakeExt.setPower(0);
        }
    }

    @Override
    public void start()
    {
        start = System.currentTimeMillis();
    }

    @Override
    public void loop()
    {
        Profiler profiler = new Profiler().disable();

        profiler.start("driveWheels()");
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        double mult = 1;
        if (slow) mult = 0.4;
        if (prevLeftJoy != leftPower)
        {
            prevLeftJoy = leftPower;
            robot.leftFront.setPower(leftPower * mult);
            robot.leftRear.setPower(leftPower * mult);
        }
        if (prevRightJoy != rightPower)
        {
            prevRightJoy = rightPower;
            robot.rightFront.setPower(rightPower * mult);
            robot.rightRear.setPower(rightPower * mult);
        }
        if (buttonHelper_1.pressing(ButtonHelper.right_stick_button))
        {
            slow = !slow;
        }
        profiler.end();

        profiler.start("dunkLogic()");
        //        if (robot.liftLimitUp.pressed())
//        {
//            robot.dunkLiftController.stopHolding();
//        }
//        else
//        {
//            robot.dunkLiftController.resumeHolding();
//        }
        if (gamepad2.left_trigger > 0.5)
        {
            robot.dunkLiftController.hold(0);
        }
        else if (gamepad2.right_trigger > 0.5)
        {
            robot.dunkLiftController.hold(robot.lift_up);
        }
        profiler.end();

        profiler.start("drivePullup()");
        if (robot.pullupLimit.pressed() && limit_press_count < 50) limit_press_count++;
        else limit_press_count = 0;

        if (buttonHelper_2.pressed(ButtonHelper.left_bumper) && (!robot.pullupLimit.pressed() || limit_press_count < 50))
        {
            robot.pullUp.setPower(1);
        }
        else if (buttonHelper_2.pressed(ButtonHelper.right_bumper))
        {
            robot.pullUp.setPower(-1);
        }
        else
        {
            if (robot.pullUp.getPower() != 0) robot.pullUp.setPower(0);
        }
        profiler.end();

        profiler.start("dunk()");
        if (robot.liftLimitDown.pressed()) robot.dunk.setPosition(robot.dunk_min);
        if (buttonHelper_1.pressing(ButtonHelper.b))
        {
            liftingDunk = false;
            droppingDunk = false;
            if (robot.dunk.getPosition() < robot.dunk_up - 0.1)
                robot.dunk.setPosition(robot.dunk_up);
            else
            {
                robot.dunk.setPosition(robot.dunk_dunk);
                scheduler.add("Pull Back Dunk",750, () -> robot.dunk.setPosition(robot.dunk_up));
            }
        }
        profiler.end();

        profiler.start("hook()");
        if (buttonHelper_2.pressing(ButtonHelper.x))
        {
            if (robot.hook.getPosition() < robot.HOOK_CLOSED)
                robot.hook.setPosition(robot.HOOK_CLOSED);
            else
                robot.hook.setPosition(robot.HOOK_OPEN);
        }
        profiler.end();

        profiler.start("driveIntake()");
        if (buttonHelper_1.pressed(ButtonHelper.left_bumper))
        {
            robot.intake.setPower(0.92);
        }
        else if (buttonHelper_1.pressed(ButtonHelper.right_bumper))
        {
            robot.intake.setPower(-0.82);
        }
        else
        {
            robot.intake.setPower(0);
        }
        profiler.end();

        profiler.start("runPivot()");
        if (buttonHelper_2.pressing(ButtonHelper.dpad_up))
        {
            robot.intakePivot.setPosition(robot.pivot_up);
        }
        else if (buttonHelper_2.pressing(ButtonHelper.dpad_down))
        {
            robot.intakePivot.setPosition(robot.pivot_down);
        }
        profiler.end();

        profiler.start("driveExtension()");
        //        if (robot.intakeLimit.pressed())
//        {
//            robot.intakeExtController.stopHolding();
//            return;
//        }
        int newPos = (int)(robot.intakeExtController.getTargetPosition() + 200.0 * (gamepad1.right_trigger - gamepad1.left_trigger));
        newPos = (int)Utils.constrain(newPos, 0, robot.ext_max); // Keep the robot from extending too far
        robot.intakeExtController.hold(newPos);
        telemetry.addData("Intake holding", robot.intakeExtController.getTargetPosition());
        telemetry.addData("Intake at", robot.intakeExtController.getCurrentPosition());
        profiler.end();

        profiler.start("slowDunk()");
        if (liftingDunk)
        {
            if (robot.dunk.getPosition() > robot.dunk_up) robot.dunk.setPosition(robot.dunk.getPosition() - 0.03);
            else liftingDunk = false;
//            robot.dunk.setPosition(Robot.dunk_up);
//            liftingDunk = false;
        }
        else if (droppingDunk)
        {
            if (robot.dunk.getPosition() < dunk_nearly_down) robot.dunk.setPosition(dunk_nearly_down);
            else if (robot.dunk.getPosition() < robot.dunk_min) robot.dunk.setPosition(robot.dunk.getPosition() + 0.05);
            else droppingDunk = false;
        }
        profiler.end();

        profiler.finish();

        scheduler.update();
        telemetry.addData("Time", Utils.elapsedTime(System.currentTimeMillis() - start));
        telemetry.addData("Slow", slow);
        telemetry.addData("Intake extension limit switch", robot.intakeLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Lower limit switch", robot.liftLimitDown.pressed() ? "Pressed" : "Released");
        telemetry.addData("Upper limit switch", robot.liftLimitUp.pressed() ? "Pressed" : "Released");
        telemetry.addData("Pullup limit switch", robot.pullupLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Dunk Position", robot.dunk.getPosition());
        telemetry.addData("Lifting Dunk: " + liftingDunk + ", dropping dunk", droppingDunk);
        telemetry.addData("Loops per second", fps);
        int[] stats = cpustat.getStats();
        telemetry.addData("CPU usage | user", stats[VMStats.CPU_USER])
                .addData("system", stats[VMStats.CPU_SYS])
                .addData("waiting", stats[VMStats.CPU_WAIT])
                .addData("idle", stats[VMStats.CPU_IDLE]);
        framecount++;
        if (System.currentTimeMillis() - lastFrame > 1000)
        {
            lastFrame = System.currentTimeMillis();
            fps = framecount;
            framecount = 0;
        }
    }

    @Override
    public void stop()
    {
        robot.uninitialize();
        cpustat.close();
        Logger.close();
    }
}
