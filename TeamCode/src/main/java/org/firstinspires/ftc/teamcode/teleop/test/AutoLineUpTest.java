package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.PIDController;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

@TeleOp(name="Automatic line-up test")
public class AutoLineUpTest extends MainTeleOp
{
    private boolean lineUp = false;
    private boolean lineup_old = false;
    private ButtonHelper buttonHelper_1;
    private Robot robot;
    private PIDController controller;

    @Override
    public void init()
    {
        super.init();
        buttonHelper_1 = new ButtonHelper(gamepad1);
        lineUp = false;
        lineup_old = false;
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        // Quick and easy proportional control
        controller = new PIDController(.01, 0, 0);
    }

    private void doLineup()
    {
        double diff = robot.rightRange.getDistance() - robot.leftRange.getDistance();
        telemetry.addData("Lining up", "");
        telemetry.addData("Difference", diff);

        double turn = controller.process(diff);
        // Positive output should increase input by turning counterclockwise
        robot.rightFront.setPower(turn);
        robot.leftFront.setPower(-turn);
    }

    @Override
    public void loop()
    {
        if (lineUp)
        {
            if (!lineup_old)
            {
                lineup_old = true;
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);

            }

            if (buttonHelper_1.pressing(ButtonHelper.y))
            {
                lineUp = false;
                return;
            }
            doLineup();
        }
        else
        {
            if (lineup_old)
            {
                lineup_old = false;
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
            }
            // Return the control to the user
            super.loop();
        }
    }
}
