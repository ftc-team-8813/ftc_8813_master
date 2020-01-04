package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;

@TeleOp(name="Acceleration Curve Test")
public class DistanceTest extends BaseAutonomous
{
    
    private Robot robot;
    private Drivetrain drivetrain;
    private PIDMotor senseWheel;
    private Logger log = new Logger("Acceleration Test");
    private volatile String status = "Waiting";
    
    private static final int FORWARD = 0;
    private static final int STRAFE = 1;
    private static final int TURN = 2;
    
    private void waitForKey() throws InterruptedException
    {
        status = "Waiting";
        while (!gamepad1.b)
        {
            Thread.sleep(10);
        }
    }
    
    private void returnHome(int direction) throws InterruptedException
    {
        status = "Return Home";
        if (direction == FORWARD)
        {
            robot.drivetrain.move(0.2, 0, 0, -senseWheel.getCurrentPosition());
        }
        else if (direction == STRAFE)
        {
            robot.drivetrain.move(0, 0.2, 0, -senseWheel.getCurrentPosition());
        }
        else if (direction == TURN)
        {
            robot.drivetrain.move(0, 0, 0.2, -senseWheel.getCurrentPosition());
        }
        Thread.sleep(1000);
    }
    
    private void curveTurn(double speed, long msecs) throws InterruptedException
    {
        drivetrain.drive(speed, 0, -speed*1.3);
        Thread.sleep(msecs);
        drivetrain.stop();
    }
    
    @Override
    public void run() throws InterruptedException
    {
        robot = Robot.instance();
        drivetrain = robot.drivetrain;
        senseWheel = drivetrain.leftBack;
        
        GlobalDataLogger.instance().addChannel("Test Status", () -> status);
        status = "Curve Turn";
        curveTurn(0.3, 250);
        
    }
}
