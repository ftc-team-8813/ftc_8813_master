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
@TeleOp(name = "MovementTest")
public class MovementTest extends OpMode
{
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
        /*if (gamepad1.left_stick_y < 0){
            robot.frontWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_y);
            robot.backWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_y);
            telemetry.addData("Front Upper", robot.frontWheel.getUpperPos());
            telemetry.addData("Back Lower", robot.backWheel.getLowerPos());
        }else if (gamepad1.left_stick_y > 0){
            robot.frontWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_y);
            robot.backWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_y);
        }else if (gamepad1.right_stick_x > 0){
            robot.frontWheel.drive(gamepad1.right_stick_x, gamepad1.right_stick_x);
            robot.backWheel.drive(gamepad1.right_stick_x, gamepad1.right_stick_x);
        }else if (gamepad1.right_stick_x < 0){
            robot.frontWheel.drive(gamepad1.right_stick_x, gamepad1.right_stick_x);
            robot.backWheel.drive(gamepad1.right_stick_x, gamepad1.right_stick_x);
        }else{
            robot.frontWheel.drive(0, 0);
            robot.backWheel.drive(0, 0);
        }*/

        /*robot.frontWheel.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        robot.frontWheel.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y);*/

        telemetry.addData("Y-Axis", gamepad1.left_stick_y);
        telemetry.addData("X-Axis", gamepad1.left_stick_x);
    }

    @Override
    public void stop()
    {
        robot.uninitialize();
        Logger.close();
    }
}
