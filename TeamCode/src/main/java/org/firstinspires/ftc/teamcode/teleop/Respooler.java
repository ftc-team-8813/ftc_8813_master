package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;

@TeleOp(name="Respooler")
public class Respooler extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Robot robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        waitForStart();
        int i = 0;
        robot.pullUp.setPower(-1);
        while (opModeIsActive() && i < 50)
        {
            if (robot.pullupLimit.pressed()) i++;
            else i = 0;
        }
        robot.pullUp.setPower(0);
    }
}
