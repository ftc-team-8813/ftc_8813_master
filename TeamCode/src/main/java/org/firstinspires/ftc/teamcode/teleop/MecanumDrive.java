package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends BaseTeleOp
{
    private static final int SPEED_SLOW = 0;
    private static final int SPEED_FAST = 1;
    private static final int SPEED_LUDICROUS = 2;
    private static final String[] speed_modes = {"Slow", "Fast", "Ludicrous"};


    private ButtonHelper buttonHelper;
    private int speed_mode = SPEED_FAST;
    
    @Override
    public void init()
    {
        super.init();
        buttonHelper = new ButtonHelper(gamepad1);
        robot.newarm.resetArm();
        robot.slide.slidemotor.setPower(0.5);
    }
    
    @Override
    public void doLoop()
    {
        if (buttonHelper.pressing(ButtonHelper.y))
        {
            if (speed_mode == SPEED_SLOW) speed_mode = SPEED_FAST;
            else speed_mode = SPEED_SLOW;
        }
        if (buttonHelper.pressing(ButtonHelper.x))
        {
            if (speed_mode == SPEED_LUDICROUS) speed_mode = SPEED_FAST;
            else speed_mode = SPEED_LUDICROUS;
        }
    
        if (gamepad1.right_bumper)
        {
            robot.intake.collectStone(0.3);
        } else if (gamepad1.left_bumper)
        {
            robot.intake.releaseStone(0.3);
        } else
        {
            robot.intake.stopIntake();
        }

        robot.newarm.moveArm(-gamepad2.right_stick_y);
        if (gamepad2.start){
            robot.newarm.resetArm();
        }
    
        if (gamepad2.dpad_down)
        {
            robot.slide.raiseLift(0, -1);
        }
        else
        {
            robot.slide.raiseLift(-gamepad2.left_stick_y);
        }

        double[] speeds;
        if (speed_mode == SPEED_SLOW)      speeds = new double[] {0.3, 0.3, 0.15}; // SLOW
        else if (speed_mode == SPEED_FAST) speeds = new double[] {0.5, 0.5, 0.5 }; // FAST
        else                               speeds = new double[] {1,   1,   1   }; // LUDICROUS

        robot.drivetrain.drive(-gamepad1.left_stick_y * speeds[0],
                                  gamepad1.left_stick_x * speeds[1],
                                 -gamepad1.right_stick_y * speeds[2]);


        if (gamepad2.a)
        {
            robot.claw.closeClaw();
        }
        else if (gamepad2.y)
        {
            robot.claw.openClaw();
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

        telemetry.addData("Speed Mode", speed_modes[speed_mode]);
        telemetry.addData("Extender Top Limit", robot.slide.getTopLimit());
        telemetry.addData("Extender Pos", robot.slide.getCurrentPos());
        telemetry.addData("Forward Ticks Moved", robot.drivetrain.leftFront.getCurrentPosition());
        // telemetry.addData("Claw Pos", robot.claw.getExtension().getPosition());
        telemetry.addData("Back Limit", robot.backSwitch.pressed());
        telemetry.addData("Claw Pos", robot.newarm.motorArm.getCurrentPosition());
    }

    public void stop(){
        robot.slide.slidemotor.getMotor().setTargetPosition(0);
        robot.slide.slidemotor.setPower(0.25);
        robot.slide.slidemotor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.slidemotor.setPower(0);
    
        super.stop();
    }
}
