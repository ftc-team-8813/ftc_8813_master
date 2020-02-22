package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Lift;
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
@TeleOp(name = "MovementTest")
public class MovementTest extends OpMode{

    protected Robot robot;
    protected Logger log;

    @Override
    public void init()
    {
        try { Logger.init(); } catch (IOException e) { throw new RuntimeException(e); }
        log = new Logger("Driver Control");
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            robot.slide.oldRaiseLift(0.05);
        } else if (gamepad1.dpad_down){
            robot.slide.oldRaiseLift(-0.05);
        } else {
            robot.slide.oldRaiseLift(0);
        }
        telemetry.addData("Mode", robot.slide.slidemotor.getMotor().getMode());
        telemetry.addData("Power", robot.slide.slidemotor.getMotor().getPower());
        telemetry.addData("Position", robot.slide.slidemotor.getCurrentPosition());
        telemetry.addData("Target", robot.slide.slidemotor.getTargetPosition());
    }


    @Override
    public void stop()
    {
        robot.uninitialize();
        Logger.close();
    }
}
