package org.firstinspires.ftc.teamcode.teleop;

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
    }
}
