package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.util.Config;

@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        MotorController mc = new MotorController(hardwareMap.dcMotor.get("flipper"), config);
        mc.hold(150);
        Thread.sleep(1200);
        for (int i = 0; i < 75; i++)
        {
            left.setPower(-(double)i / 100);
            right.setPower((double)i / 100);
            Thread.sleep(10);
        }
        Thread.sleep(250);
        left.setPower(0);
        right.setPower(0);
        mc.hold(700);
        Thread.sleep(3000);
        for (int i = 0; i < 50; i++)
        {
            left.setPower((double)i / 100);
            right.setPower(-(double)i / 100);
            Thread.sleep(10);
        }
        left.setPower(0);
        right.setPower(0);
        mc.close();
    }
}
