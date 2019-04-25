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
    private boolean intakeIn = true;
    private boolean landerMode = false;
    private int limit_press_count = 0;
    private int prevLiftPos = 0;
    private double prevAutoPivotPos;

    private double dunk_nearly_down;

    private Scheduler scheduler = new Scheduler();
    private Logger log;

    private long start;

    private int fps = 0;
    private int framecount;
    private long lastFrame = 0;
    private boolean manual_pivot = false;

    private VMStats cpustat;

    private Scheduler.TaskCallback respoolerHandle;

    
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
        //
        // cpustat = new VMStats(1);
        robot.dunkLiftController.stopHolding();
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
        robot.leftFront.setPower(leftPower * mult);
        robot.leftRear.setPower(leftPower * mult);
        robot.rightFront.setPower(rightPower * mult);
        robot.rightRear.setPower(rightPower * mult);
        if (buttonHelper_1.pressing(ButtonHelper.right_stick_button))
        {
            slow = !slow;
        }
        profiler.end();

        profiler.start("dunkLogic()");
//        if (robot.liftLimitDown.pressed())
//        {
//            robot.dunkLiftController.constrainPower(0, 1);
//        }
//        else
//        {
//            robot.dunkLiftController.constrainPower(-0.05, 1);
//        }
//        if (gamepad2.left_trigger > 0.5)
//        {
//            robot.dunkLiftController.hold(0);
//            robot.dunk.setPosition(robot.dunk_min);
//        }
//        else if (gamepad2.right_trigger > 0.5)
//        {
//            robot.dunkLiftController.hold(robot.lift_up);
//        }
        double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;
        robot.dunkLift.setPower(liftPower);
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
        if (robot.liftLimitDown.pressing()) robot.dunk.setPosition(robot.dunk_min);
        if (robot.intakeExtController.getCurrentPosition() < 100 && robot.dunkLift.getCurrentPosition() < 200)
            robot.dunk.setPosition(robot.dunk_min);
        if (buttonHelper_1.pressing(ButtonHelper.b) && !robot.liftLimitDown.pressed())
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
            robot.intake.setPower(0.85);
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
        double manualPivotPos = robot.intakePivot.getPosition();
        if (gamepad2.dpad_up)
        {
            manualPivotPos = robot.pivot_up;
            manual_pivot = true;
        }
        else if (gamepad2.dpad_down)
        {
            manualPivotPos = robot.pivot_down;
            manual_pivot = true;
        }
        profiler.end();

        profiler.start("driveExtension()");
        //        if (robot.intakeLimit.pressed())
//        {
//            robot.intakeExtController.stopHolding();
//            return;
//        }
        int oldPos = robot.intakeExtController.getTargetPosition();
        int newPos = (int)(oldPos + 200.0 * (-gamepad2.right_stick_y));
        newPos = (int)Utils.constrain(newPos, 0, robot.ext_max); // Keep the robot from extending too far
        double pivotPos = robot.intakePivot.getPosition();
        double autoPivotPos = robot.intakePivot.getPosition();
        // Pop the dunk up when moving the intake
        if (newPos != oldPos)
        {
            robot.dunk.setPosition(robot.dunk_up);
        }
        robot.intakeExtController.hold(newPos);
        if (Utils.floatEquals(robot.intakePivot.getPosition(), robot.pivot_up))
        {
            if (robot.intakeExtController.getCurrentPosition() > robot.ext_drop + 100)
            {
                autoPivotPos = robot.pivot_down;
            }
        }
        else
        {
            if (robot.intakeExtController.getCurrentPosition() < robot.ext_drop - 100)
            {
                autoPivotPos = robot.pivot_up;
            }
        }

        // Force pivot up when retracting
        if (newPos < oldPos)
        {
            manual_pivot = true; // Pretend to be manual pivot
            manualPivotPos = robot.pivot_up;
        }

        if (manual_pivot)
        {
            if (!Utils.floatEquals(autoPivotPos, prevAutoPivotPos))
            {
                pivotPos = autoPivotPos;
                manual_pivot = false;
            }
            else if (!Utils.floatEquals(manualPivotPos, pivotPos))
            {
                pivotPos = manualPivotPos;
            }
        }
        else
        {
            pivotPos = autoPivotPos;
        }
        robot.intakePivot.setPosition(pivotPos);
        prevAutoPivotPos = autoPivotPos;
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

        profiler.start("landerMode()");
        if (buttonHelper_1.pressing(ButtonHelper.y))
        {
            log.d("Trigger lander mode");
            if (!landerMode)
            {
                log.d("Enable lander mode");
                manual_pivot = true;
                landerMode = true;
                robot.hook.setPosition(robot.HOOK_OPEN);
                robot.intakePivot.setPosition(robot.pivot_down);
                robot.pullUp.setPower(-1);
                if (respoolerHandle != null) respoolerHandle.cancel();
                respoolerHandle = scheduler.add(500, () ->
                {
                    if (robot.pullupLimit.pressed())
                    {
                        robot.pullUp.setPower(0);
                        respoolerHandle.cancel();
                    }
                }, true);
            }
            else
            {
                log.d("Disable lander mode");
                landerMode = false;
                if (respoolerHandle != null) respoolerHandle.cancel();
                robot.pullUp.setPower(1);
                respoolerHandle = scheduler.add(500, () ->
                {
                    robot.pullUp.setPower(0);
                    manual_pivot = false;
                });
            }
        }
        profiler.end();

        if (liftPower != 0 && Utils.floatEquals(robot.dunk.getPosition(), robot.dunk_up))
        {
            robot.dunk.setPosition(robot.dunk_min);
        }

        profiler.finish();

        scheduler.update();
        telemetry.addData("Time", Utils.elapsedTime(System.currentTimeMillis() - start));
        telemetry.addData("Lander Mode", landerMode);
        telemetry.addData("Manual pivot", manual_pivot);
        telemetry.addData("Pivot pos", "auto: " + autoPivotPos + " manual: " + manualPivotPos + " actual: " + pivotPos);
        telemetry.addData("Slow", slow);
        telemetry.addData("Intake pivot", robot.intakePivot.getPosition());
        telemetry.addData("Intake extension limit switch", robot.intakeLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Lower limit switch", robot.liftLimitDown.pressed() ? "Pressed" : "Released");
        telemetry.addData("Upper limit switch", robot.liftLimitUp.pressed() ? "Pressed" : "Released");
        telemetry.addData("Pullup limit switch", robot.pullupLimit.pressed() ? "Pressed" : "Released");
        telemetry.addData("Dunk Position", robot.dunk.getPosition());
        telemetry.addData("Lifting Dunk: " + liftingDunk + ", dropping dunk", droppingDunk);
        telemetry.addData("Loops per second", fps);
//        int[] stats = cpustat.getStats();
//        telemetry.addData("CPU usage | user", stats[VMStats.CPU_USER])
//                .addData("system", stats[VMStats.CPU_SYS])
//                .addData("waiting", stats[VMStats.CPU_WAIT])
//                .addData("idle", stats[VMStats.CPU_IDLE]);
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
        // cpustat.close();
        Logger.close();
    }
}
