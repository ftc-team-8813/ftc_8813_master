package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;

@TeleOp(name="Swerve Test/Calibration")
public class SwerveTest extends OpMode
{
    private Robot robot;
    private final String[] modes =
            {
                    "Direct Drive",
                    "Forward Only",
                    "Turn Only"
            };
    private final String[][] descriptions =
            {
                    {
                        "Joysticks run the front wheel",
                        "Triggers run the back wheel",
                        "Bumpers invert the direction of the triggers"
                    },
                    {
                        "Left joystick drives the front wheel",
                        "Right joystick drives the back wheel"
                    },
                    {
                        "Left joystick drives the front wheel",
                        "Right joystick drives the back wheel"
                    }
            };
    private int mode = 0;
    private ButtonHelper buttons;
    
    @Override
    public void init()
    {
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        buttons = new ButtonHelper(gamepad1);
    }
    
    private void drive()
    {
        switch (mode)
        {
            case 0:
            {
                robot.frontWheel.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            
                int lmult = (gamepad1.left_bumper ? 0 : 1) * 2 - 1;
                int rmult = (gamepad1.right_bumper ? 0 : 1) * 2 - 1;
                robot.backWheel.drive(gamepad1.left_trigger * lmult, gamepad1.right_trigger * rmult);
                break;
            }
            case 1:
            {
                robot.frontWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_y);
                robot.backWheel.drive(-gamepad1.right_stick_y, gamepad1.right_stick_y);
                break;
            }
            case 2:
            {
                robot.frontWheel.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_y);
                robot.backWheel.drive(-gamepad1.right_stick_y, -gamepad1.right_stick_y);
                break;
            }
        }
    }
    
    private void updateMode()
    {
        if (buttons.pressing(ButtonHelper.dpad_up))
        {
            mode--;
            if (mode < 0) mode += modes.length;
        }
        else if (buttons.pressing(ButtonHelper.dpad_down))
        {
            mode++;
            if (mode >= modes.length) mode -= modes.length;
        }
    }
    
    private void display()
    {
        telemetry.clearAll();
        telemetry.addData("Front wheel", "%d, %d", robot.frontWheel.getUpperPos(), robot.frontWheel.getLowerPos());
        telemetry.addData("Rear wheel", "%d, %d", robot.backWheel.getUpperPos(), robot.backWheel.getLowerPos());
        telemetry.addData("Mode", modes[mode]);
    }
    
    @Override
    public void loop()
    {
        updateMode();
        drive();
        display();
    }
    
    // 1 rotation forward = 300 encoders
    // 1 rotation turn = 1440 encoders -> 1/4 rotation = 360 encoders
}
