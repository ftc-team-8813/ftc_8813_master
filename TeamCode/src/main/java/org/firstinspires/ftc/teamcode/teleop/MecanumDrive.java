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
    
    private boolean liftTrigger = false;
    
    private long start;
    
    private int updateCount;
    private long lastLog;

    private long lastTelemetry;
    
    private Profiler profiler;
    private static final boolean PROFILE = false;
    
    @Override
    public void init()
    {
        super.init();
        buttonHelper = new ButtonHelper(gamepad1);
        robot.newarm.resetArm();
        
        robot.slide.slidemotor.setPower(0.5);
        // robot.imu.stop();
        // robot.drivetrain.enableAngleCorrection();
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
        robot.drivetrain.enableAsyncLoop();
        
        profiler = new Profiler();
        
        // Bulk caching is broken internally for digital channels
        // robot.leftHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        // robot.rightHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    
    public void start()
    {
        super.start();
        start = System.currentTimeMillis();
    }
    
    private void setSpeedMode(int mode)
    {
        speed_mode = mode;
        if (speed_mode == SPEED_SLOW)           robot.rightLed.setColor(64, 0,    255);
        else if (speed_mode == SPEED_FAST)      robot.rightLed.setColor(0,   255, 0);
        else if (speed_mode == SPEED_FASTER)    robot.rightLed.setColor(255, 0,   0);
        else if (speed_mode == SPEED_LUDICROUS) robot.rightLed.setColor(200, 255, 225);
    }
    
    @Override
    public void doLoop()
    {
        if (PROFILE) profiler.start("loop()");
        if (PROFILE) profiler.start("speed");
        if (buttonHelper.pressing(ButtonHelper.y))
        {
            if (speed_mode == SPEED_SLOW)
            {
                setSpeedMode(SPEED_FAST);
            }
            else
            {
                setSpeedMode(SPEED_SLOW);
            }
        }
        if (buttonHelper.pressing(ButtonHelper.x))
        {
            if (speed_mode == SPEED_FASTER || speed_mode == SPEED_LUDICROUS)
            {
                
                setSpeedMode(SPEED_FAST);
            }
            else
            {
                if (gamepad1.start)
                {
                    setSpeedMode(SPEED_LUDICROUS);
                }
                else
                {
                    setSpeedMode(SPEED_FASTER);
                }
            }
        }
        if (PROFILE) profiler.end();
    
        if (PROFILE) profiler.start("intake");
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
            robot.intake.collectStone(0.3);
            intakeTime = System.currentTimeMillis();
        }
        else
        {
            robot.intake.stopIntake();
            intakeTrigger = false;
        }
        if (PROFILE) profiler.end();
    
        if (PROFILE) profiler.start("arm");
        robot.newarm.moveArm(-gamepad2.right_stick_y * 0.6);
        if (gamepad2.start && !gamepad2.b){
            robot.newarm.resetArm();
        }
        if (PROFILE) profiler.end();
    
        if (PROFILE) profiler.start("lift");
        if (gamepad2.dpad_down)
        {
            robot.slide.raiseLift(0, -1);
        }
        else
        {
            robot.slide.raiseLift(-gamepad2.left_stick_y);
        }
        
        if (speed_mode != SPEED_SLOW && !robot.bottomlimit.pressed() && !liftTrigger)
        {
            liftTrigger = true;
            setSpeedMode(SPEED_SLOW);
        }
        if (robot.bottomlimit.pressing())
        {
            liftTrigger = false;
            setSpeedMode(SPEED_FAST);
        }
        if (PROFILE) profiler.end();
    
        if (PROFILE) profiler.start("drivetrain");
        double[] speeds;
        if (speed_mode == SPEED_SLOW)        speeds = new double[] {0.3, 0.2, 0.15}; // SLOW
        else if (speed_mode == SPEED_FAST)   speeds = new double[] {0.5, 0.5, 0.5 }; // FAST
        else if (speed_mode == SPEED_FASTER) speeds = new double[] {0.8, 0.8, 0.8 }; // FASTER
        else                                 speeds = new double[] {1,   1,   1   }; // LUDICROUS
        
        if (PROFILE) profiler.start("snap");
        // Snap to 90-degree angles
        if (gamepad1.right_stick_button)
        {
            if (robot.drivetrain.getAngleInfluence() == 0)
            {
                double currHeading = robot.imu.getHeading();
                double snapAngle = Math.round(currHeading / 90) * 90;
                log.i("Snap to %.0f", snapAngle);
                robot.drivetrain.setTargetAngle(snapAngle);
                robot.drivetrain.setAngleInfluence(0.5);
            }
        }
        else
        {
            if (robot.drivetrain.getAngleInfluence() > 0)
            {
                robot.drivetrain.disableAngleCorrection();
            }
        }
        if (PROFILE) profiler.end();
        if (PROFILE) profiler.start("drive");

        robot.drivetrain.drive(-gamepad1.left_stick_y * speeds[0],
                                  gamepad1.left_stick_x * speeds[1],
                                 -gamepad1.right_stick_y * speeds[2]);
        if (PROFILE) profiler.end();
        if (PROFILE) profiler.end();
        // robot.drivetrain.manualLoop(); // Manually update the drivetrain
    
    
        if (PROFILE) profiler.start("claw");
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
        if (PROFILE) profiler.end();
    
    
        if (PROFILE) profiler.start("hook");
        if (gamepad2.x){
            robot.foundationhook.moveHookDown();
        }else if (gamepad2.b){
            robot.foundationhook.moveHookUp();
        }
        if (PROFILE) profiler.end();
    
        if (PROFILE) profiler.start("linkage");
        if (gamepad1.dpad_up){
            robot.intakelinkage.moveLinkageOut();
        } else if (gamepad1.dpad_down){
            robot.intakelinkage.moveLinkageIn();
        }
        if (PROFILE) profiler.end();
        
        if (buttonHelper.pressing(ButtonHelper.back))
        {
            if (robot.drivetrain.isFieldCentric())
            {
                robot.drivetrain.disableFieldCentric();
            }
            else
            {
                robot.drivetrain.enableFieldCentric();
            }
        }
    
        if (PROFILE) profiler.start("telemetry");
        if (System.currentTimeMillis() - lastTelemetry > 750)
        {
            lastTelemetry = System.currentTimeMillis();
            telemetry.addData("Time", Utils.elapsedTime(System.currentTimeMillis() - start));
            // 2 min == 120s == 120000ms
            long remaining = 120000 - (System.currentTimeMillis() - start);
            if (remaining < 60000 && timeTrigger <= 0)
            {
                timeTrigger = 1;
                log.i("1 minute remaining");
                // telemetry.speak("One minute remaining");
            } else if (remaining < 30000 && timeTrigger <= 1)
            {
                timeTrigger = 2;
                log.i("30 seconds remaining");
                // telemetry.speak("Thirty seconds remaining");
            } else if (remaining < 10000 && timeTrigger <= 2)
            {
                timeTrigger = 3;
                log.i("10 seconds remaining");
                // telemetry.speak("Ten seconds remaining");
            } else if (remaining < 0 && timeTrigger <= 3)
            {
                timeTrigger = 4;
                log.w("Time!!!");
            }
    
            telemetry.addData("Speed Mode", speed_modes[speed_mode]);
            telemetry.addData("IMU status", robot.imu.getDetailStatus());
            telemetry.addData("Field Centric", robot.drivetrain.isFieldCentric());
            telemetry.addData("Heading", robot.imu.getHeading());
            telemetry.addData("Lift Position", robot.slide.getCurrentPos());
            telemetry.addData("Bottom limit", robot.bottomlimit.pressed());
            // telemetry.addData("Claw Pos", robot.claw.getExtension().getPosition());
            telemetry.addData("Back Limit", robot.backSwitch.pressed());
            telemetry.addData("Claw Pos", robot.newarm.motorArm.getCurrentPosition());
        }
        if (PROFILE) profiler.end();
        
        if (PROFILE) profiler.finish();
    
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
