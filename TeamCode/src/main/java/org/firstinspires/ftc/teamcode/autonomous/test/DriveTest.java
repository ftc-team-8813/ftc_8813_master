package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Drive Test")
public class DriveTest extends BaseAutonomous
{

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        telemetry.setAutoClear(false);


        ////////////////////////////////////////////////////////////////////////////////////////////
        // TEST 0: Driving forwards and backwards

        Telemetry.Item test_line = telemetry.addData("Test 0, Part 0", "Drive forward 12 inches @ 50% power");
        /*
        countdown(3);

        robot.forward(12, .5);

        test_line.setCaption("Test 0, Part 1");
        test_line.setValue("Drive backward 12 inches at 50% power");
        countdown(3);

        robot.reverse(12, .5);

        test_line.setCaption("Test 0, Part 2");
        test_line.setValue("Drive forward 1 inch at 25% power");
        countdown(3);

        robot.forward(1, .25);

        test_line.setCaption("Test 1");
        test_line.setValue("");
        countdown(10);
        */
        test_line.setCaption("Test 1, Part 0");
        test_line.setValue("Point-turn 360 degrees");
        countdown(3);

        robot.turn(2 * Math.PI, 0, .5);

        test_line.setCaption("Test 1, Part 1");
        test_line.setValue("Point-turn 180 degrees counterclockwise");
        countdown(3);

        robot.turn(-Math.PI, 0, .5);

        test_line.setCaption("Test 1, Part 2");
        test_line.setValue("Point turn accuracy test -- 8x 90 degree turns at 25% power");
        countdown(3);

        for (int i = 0; i < 8; i++)
        {
            robot.turn(Math.PI / 2, 0, .5);
            Thread.sleep(1000);
        }

        test_line.setCaption("Test 1, Part 3");
        test_line.setValue("Point turn speed accuracy test -- 4x 90 degree turns; 25% power increments");
        countdown(3);

        for (int i = 0; i < 4; i++)
        {
            robot.turn(Math.PI / 2, 0, .25 + .25 * i);
            Thread.sleep(1000);
        }
    }

    private void countdown(int seconds) throws InterruptedException
    {
        Telemetry.Item item = telemetry.addData("Starting in", seconds);
        for (int i = seconds; i > 0; i--)
        {
            item.setValue(i);
            telemetry.update();
            Thread.sleep(1000);
        }
        telemetry.removeItem(item);
        telemetry.update();
    }
}
