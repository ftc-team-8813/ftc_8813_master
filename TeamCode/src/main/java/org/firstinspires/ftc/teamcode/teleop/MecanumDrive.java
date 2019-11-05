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
    public void loop()
    {
        if (buttonHelper.pressing(ButtonHelper.x))
            slow = !slow;
        double mult = 1;
        if (slow) mult = 0.4;
        robot.drivetrain.drive(-gamepad1.left_stick_y * mult, gamepad1.left_stick_x * mult, -gamepad1.right_stick_y * mult);
        robot.slide.raiseLift(-gamepad2.left_stick_y);
        if (gamepad2.left_bumper){
            robot.intake.collectStone(1);
        }else if (gamepad2.right_bumper){
            robot.intake.releaseStone(1);
        }else{
            robot.intake.stopIntake();
        }
        robot.arm.extend(-gamepad2.right_stick_y * 0.005);
        if (gamepad2.a)
        {
            robot.arm.closeClaw();
        }
        else if (gamepad2.y)
        {
        robot.slide.raiseLift(-gamepad2.left_stick_y);
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
