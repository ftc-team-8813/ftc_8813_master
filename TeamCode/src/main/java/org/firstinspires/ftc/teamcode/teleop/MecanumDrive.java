package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends BaseTeleOp
{
    
    private ButtonHelper buttonHelper;
    private boolean slow;
    
    @Override
    public void init()
    {
        super.init();
        buttonHelper = new ButtonHelper(gamepad1);
    }
    
    @Override
    public void doLoop()
    {
        if (buttonHelper.pressing(ButtonHelper.x))
            slow = !slow;
        if (gamepad2.left_bumper){
            robot.intake.collectStone(1);
        }else if (gamepad2.right_bumper){
            robot.intake.releaseStone(1);
        }else{
            robot.intake.stopIntake();
        }
        robot.arm.extend(-gamepad2.right_stick_y * 0.005);
        if (slow)
            robot.drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_y);
        else
            robot.drivetrain.drive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, -gamepad1.right_stick_y * 0.3);
        robot.slide.raiseLift(-gamepad2.left_stick_y * 0.4);
        robot.arm.extend(-gamepad2.right_stick_y * 0.004);
        if (gamepad2.a)
        {
            robot.arm.closeClaw();
        }
        else if (gamepad2.y)
        {
            robot.arm.openClaw();
        }
        if (gamepad2.x){
            robot.foundationHook.moveHookDown();
        }
        if (gamepad2.b){
            robot.foundationHook.moveHookUp();
        }
    }
}
