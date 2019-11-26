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
        telemetry.addData("Extender Top Limit", robot.slide.getTopLimit());
        telemetry.addData("Extender Pos", robot.slide.getCurrentPos());
        telemetry.addData("Forward Ticks Moved", robot.drivetrain.leftFront.getCurrentPosition());
        telemetry.addData("Arm Pos", robot.arm.getExtension().getPosition());



        if (buttonHelper.pressing(ButtonHelper.x))
            slow = !slow;
        if (buttonHelper.pressing(ButtonHelper.y))
            superslow = !superslow;

        if (gamepad1.left_bumper){
            robot.intake.collectStone(0.5);
        }else if (gamepad1.right_bumper){
            robot.intake.releaseStone(0.5);
        }else{
            robot.intake.stopIntake();
        }


        if (!slow)
            robot.drivetrain.drive(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_y * 0.4);
        else if (!superslow)
            robot.drivetrain.drive(-gamepad1.left_stick_y * 0.2, gamepad1.left_stick_x * 0.2, -gamepad1.right_stick_y * 0.1);
        else
            robot.drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_y);

        robot.newarm.moveArm(-gamepad2.right_stick_y);

        robot.slide.raiseLift(-gamepad2.left_stick_y);


        if (gamepad2.a)
        {
            robot.arm.closeClaw();
        }
        else if (gamepad2.y)
        {
            robot.arm.openClaw();
        }


        if (gamepad2.x){
            robot.foundationhook.moveHookDown();
        }else if (gamepad2.b){
            robot.foundationhook.moveHookUp();
        }

        if (gamepad1.dpad_up){
            robot.intakelinkage.moveLinkageOut();
        } else if (gamepad1.dpad_down){
            robot.intakelinkage.moveLinkageIn();
        }
    }

    public void stop(){
        robot.slide.slidemotor.getMotor().setTargetPosition(0);
        robot.slide.slidemotor.setPower(0.25);
        robot.slide.slidemotor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.slidemotor.setPower(0);
    }
}
