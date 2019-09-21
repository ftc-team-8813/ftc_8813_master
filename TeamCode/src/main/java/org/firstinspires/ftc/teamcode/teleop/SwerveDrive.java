package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Swerve Drive")
public class SwerveDrive extends BaseTeleOp
{
    
    @Override
    public void init()
    {
        super.init();
        robot.frontWheel.resetEncoders();
        robot.backWheel.resetEncoders();
        robot.backWheel.copy(robot.frontWheel); // Experimental but works :D
    }
    
    @Override
    public void loop()
    {
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_y;
        robot.frontWheel.drive(turn + forward, turn - forward);
        // And the back wheel will follow
    }
}
