package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Autonomous(name="Turn Test")
public class TurnTest extends BaseAutonomous
{
    private Robot robot;
    
    @Override
    public void initialize() throws InterruptedException
    {
        robot = Robot.instance();
        robot.imu.initialize();
        robot.imu.waitForInit(telemetry);
        robot.imu.start();
        GlobalThreadPool.instance().start(() ->
        {
            while (true)
            {
                telemetry.addData("IMU Heading", robot.imu.getHeading());
                telemetry.update();
                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
    }
    
    @Override
    public void run() throws InterruptedException
    {
        robot.drivetrain.enableAngleCorrection();
        for (int i = 0; i < 3; i++)
        {
            robot.drivetrain.move(0, 0.7, 0, 250);
            Thread.sleep(1000);
            robot.drivetrain.move(0, 0.7, 0, -250);
            Thread.sleep(1000);
        }
        /*
        robot.drivetrain.stop();
        robot.drivetrain.setAngleInfluence(0.2);
        robot.drivetrain.setTargetAngle(360);
         */
    }
}
