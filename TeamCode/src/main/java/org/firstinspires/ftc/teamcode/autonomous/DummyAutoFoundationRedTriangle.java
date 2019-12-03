package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;


@Autonomous(name="DummyAutoFoundationRedTriangle")
public class DummyAutoFoundationRedTriangle extends BaseAutonomous
{
    public void initialize(){
        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.foundationhook.moveHookUp();

        robot.drivetrain.move(-0.4, 0, 0, tickstoInches(30));
        Thread.sleep(1000);
        robot.drivetrain.stop();

        robot.drivetrain.move(0, -0.3, 0, tickstoInches(30));
        Thread.sleep(1000);
        robot.drivetrain.stop();

        robot.foundationhook.moveHookDown();

        robot.drivetrain.move(0.4, 0, 0, tickstoInches(30));
        Thread.sleep(1000);
        robot.drivetrain.stop();

        robot.foundationhook.moveHookUp();

        robot.drivetrain.move(0, 0.6, 0, tickstoInches(65));
        Thread.sleep(1000);
        robot.drivetrain.stop();
    }

    public int tickstoInches(double dist){
        final double CIRCUMFERENCE = 3.14*2;
        double degrees = (dist/CIRCUMFERENCE)*360;
        double ticks = degrees*(537.6 /360);
        return (int) ticks;
    }
}
