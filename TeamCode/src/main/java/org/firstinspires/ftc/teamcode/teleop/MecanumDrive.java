package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends BaseTeleOp
{
    private static final int SPEED_SLOW = 0;
    private static final int SPEED_FAST = 1;
    private static final int SPEED_FASTER = 2;
    private static final int SPEED_LUDICROUS = 3;
    private static final String[] speed_modes = {"Slow", "Fast", "Faster", "Ludicrous"};


    private ButtonHelper buttonHelper;
    private int speed_mode = SPEED_FAST;
    
    private boolean intakeTrigger = false;
    private long intakeTime = 0;
    private int timeTrigger = 0;
    
    private long start;
    
    private int updateCount;
    private long lastLog;
    
    private long lastTelemetry;
    
    private Profiler profiler;
    
    @Override
    public void init()
    {
        super.init();
        buttonHelper = new ButtonHelper(gamepad1);
        robot.newarm.resetArm();
        robot.slide.slidemotor.setPower(0.5);
        // robot.imu.stop();
        // robot.drivetrain.enableAngleCorrection();
        robot.drivetrain.enableAsyncLoop();
        
        profiler = new Profiler();
    }
    
    public void start()
    {
        super.start();
        start = System.currentTimeMillis();
    }
    
    @Override
    public void doLoop()
    {
        // profiler.start("loop()");
        // profiler.start("speed");
        if (buttonHelper.pressing(ButtonHelper.y))
        {
            if (speed_mode == SPEED_SLOW) speed_mode = SPEED_FAST;
            else speed_mode = SPEED_SLOW;
        }
        if (buttonHelper.pressing(ButtonHelper.x))
        {
            if (speed_mode == SPEED_FASTER || speed_mode == SPEED_LUDICROUS) speed_mode = SPEED_FAST;
            else
            {
                if (gamepad1.start)
                    speed_mode = SPEED_LUDICROUS;
                else
                    speed_mode = SPEED_FASTER;
            }
        }
        // profiler.end();
        
        // profiler.start("intake");
        double dist = robot.centerRange.getDistance();
        if (gamepad1.right_bumper)
        {
            robot.intake.collectStone(0.3);
        }
        else if (gamepad1.left_bumper)
        {
            robot.intake.releaseStone(0.3);
        }
        
        // Automation
        else if (dist < 50)
        {
            robot.leftLed.setColor(255, 0, 0);
            if (!intakeTrigger)
            {
                intakeTrigger = true;
                
                robot.claw.closeClaw();
                robot.intakelinkage.moveLinkageIn();
            }
            if (System.currentTimeMillis() - intakeTime < 120)
            {
                robot.intake.collectStone(0.3);
            }
            else
            {
                robot.intake.stopIntake();
            }
        }
        else if (dist < 100)
        {
            int greenAmt = (int)((dist - 60) * 255 / 80);
            greenAmt = Range.clip(greenAmt, 0, 255);
            robot.leftLed.setColor(255, greenAmt, 0);
            robot.intake.collectStone(0.3);
            intakeTime = System.currentTimeMillis();
        }
        else
        {
            int redAmt = (int)((240 - dist) * 255 / 100);
            redAmt = Range.clip(redAmt, 0, 255);
            robot.leftLed.setColor(redAmt, 255, 0);
            robot.intake.stopIntake();
            intakeTrigger = false;
        }
        // profiler.end();
        
        // profiler.start("arm");
        robot.newarm.moveArm(-gamepad2.right_stick_y * .8);
        if (gamepad2.start && !gamepad2.b){
            robot.newarm.resetArm();
        }
        // profiler.end();
    
        // profiler.start("lift");
        if (gamepad2.dpad_down)
        {
            robot.slide.raiseLift(0, -1);
        }
        else
        {
            robot.slide.raiseLift(-gamepad2.left_stick_y);
        }
        // profiler.end();

        // profiler.start("drivetrain");
        double[] speeds;
        if (speed_mode == SPEED_SLOW)        speeds = new double[] {0.3, 0.3, 0.15}; // SLOW
        else if (speed_mode == SPEED_FAST)   speeds = new double[] {0.5, 0.5, 0.5 }; // FAST
        else if (speed_mode == SPEED_FASTER) speeds = new double[] {0.8, 0.8, 0.8 }; // FASTER
        else                                 speeds = new double[] {1,   1,   1   }; // LUDICROUS

        robot.drivetrain.drive(-gamepad1.left_stick_y * speeds[0],
                                  gamepad1.left_stick_x * speeds[1],
                                 -gamepad1.right_stick_y * speeds[2]);
        // profiler.end();
        // robot.drivetrain.manualLoop(); // Manually update the drivetrain


        // profiler.start("claw");
        if (gamepad2.a)
        {
            robot.claw.closeClaw();
        }
        else if (gamepad2.y)
        {
            robot.claw.openClaw();
        }
        else if (gamepad2.right_trigger > .75)
        {
            robot.claw.setClawUp();
        }
        // profiler.end();
    
    
        // profiler.start("hook");
        if (gamepad2.x){
            robot.foundationhook.moveHookDown();
        }else if (gamepad2.b){
            robot.foundationhook.moveHookUp();
        }
        // profiler.end();

        // profiler.start("linkage");
        if (gamepad1.dpad_up){
            robot.intakelinkage.moveLinkageOut();
        } else if (gamepad1.dpad_down){
            robot.intakelinkage.moveLinkageIn();
        }
        // profiler.end();
        
        // profiler.start("telemetry");
        if (System.currentTimeMillis() - lastTelemetry > 500)
        {
            lastTelemetry = System.currentTimeMillis();
            telemetry.addData("Time", Utils.elapsedTime(System.currentTimeMillis() - start));
            // 2 min == 120s == 120000ms
            long remaining = 120000 - (System.currentTimeMillis() - start);
            if (remaining < 60000 && timeTrigger <= 0)
            {
                timeTrigger = 1;
                log.i("1 minute remaining");
            } else if (remaining < 30000 && timeTrigger <= 1)
            {
                timeTrigger = 2;
                log.i("30 seconds remaining");
            } else if (remaining < 10000 && timeTrigger <= 2)
            {
                timeTrigger = 3;
                log.i("10 seconds remaining");
            } else if (remaining < 0 && timeTrigger <= 3)
            {
                timeTrigger = 4;
                log.e("Time!!!");
            }
    
            telemetry.addData("Speed Mode", speed_modes[speed_mode]);
            telemetry.addData("Extender Top Limit", robot.slide.getTopLimit());
            telemetry.addData("Extender Pos", robot.slide.getCurrentPos());
            telemetry.addData("Forward Ticks Moved", robot.drivetrain.leftFront.getCurrentPosition());
            // telemetry.addData("Claw Pos", robot.claw.getExtension().getPosition());
            telemetry.addData("Back Limit", robot.backSwitch.pressed());
            telemetry.addData("Claw Pos", robot.newarm.motorArm.getCurrentPosition());
            telemetry.addData("Heading", robot.imu.getHeading());
        }
        // profiler.end();
        
        // profiler.finish();
    
        updateCount++;
        if (System.currentTimeMillis() - lastLog > 1000)
        {
            log.d("FPS: %d", updateCount);
            updateCount = 0;
            lastLog = System.currentTimeMillis();
        }
    }

    public void stop(){
        robot.slide.slidemotor.getMotor().setTargetPosition(0);
        robot.slide.slidemotor.setPower(0.25);
        robot.slide.slidemotor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.slidemotor.setPower(0);
    
        super.stop();
    }
}
