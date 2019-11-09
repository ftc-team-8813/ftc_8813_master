package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends BaseTeleOp
{
    
    private ButtonHelper buttonHelper;
    private boolean slow;
    private boolean superslow;
    
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
        if (buttonHelper.pressing(ButtonHelper.y))
            superslow = !superslow;

        if (gamepad1.left_bumper){
            robot.intake.collectStone(1);
        }else if (gamepad1.right_bumper){
            robot.intake.releaseStone(1);
        }else{
            robot.intake.stopIntake();
        }


        robot.arm.extend(-gamepad2.right_stick_y * 0.004);

        telemetry.addData("Extender Delta", robot.slide.getCurrentPos());

        if (!slow)
            robot.drivetrain.drive(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_y * 0.4);
        else if (!superslow)
            robot.drivetrain.drive(-gamepad1.left_stick_y * 0.2, gamepad1.left_stick_x * 0.2, -gamepad1.right_stick_y * 0.1);
        else
            robot.drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_y);


        robot.slide.raiseLift(-gamepad2.left_stick_y * 0.4);


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

    public void stop(){
        robot.slide.slidemotor.getMotor().setTargetPosition(0);
        robot.slide.slidemotor.setPower(0.25);
        robot.slide.slidemotor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.slidemotor.setPower(0);
    }
}
