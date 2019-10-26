package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends BaseTeleOp
{
    @Override
    public void init()
    {
        super.init();
    }
    
    @Override
    public void loop()
    {
        robot.drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_y);
        robot.slide.raiseLift(-gamepad2.left_stick_y * 0.4);
//        robot.arm.extend(-gamepad2.right_stick_y * 0.005);
//        if (gamepad2.a)
//        {
//            robot.arm.closeClaw();
//        }
//        else if (gamepad2.y)
//        {
//            robot.arm.openClaw();
//        }
    }
}
