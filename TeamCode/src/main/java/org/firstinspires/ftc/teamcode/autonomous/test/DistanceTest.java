package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.common.util.Logger;

@Autonomous(name="Distance Test")
public class DistanceTest extends BaseAutonomous
{
    
    private Robot robot;
    private Drivetrain drivetrain;
    private Logger log = new Logger("Distance Test");
    
    private static final int LEFT_RANGE = 1;
    private static final int RIGHT_RANGE = 2;
    
    public void moveToRange(double dist, double speed, int sensor) throws InterruptedException
    {
        double origSpeed = speed;
        drivetrain.drive(speed, 0, 0);
        RangeSensor sens;
        if (sensor == LEFT_RANGE) sens = robot.leftRange;
        else sens = robot.rightRange;
        int prev_enc = 0;
        double prev_off = Double.POSITIVE_INFINITY;
        while (true)
        {
            double range = sens.getDistance();
            double off = (range - dist) * Math.signum(origSpeed);
            if (off <= 0 && off > -20)
            {
                drivetrain.stop();
                int enc = drivetrain.rightBack.getCurrentPosition();
                log.v("Overshoot correction: overshoot=%.0f, speed=%.3f, enc delta=%d", -off, speed, enc - prev_enc);
                if (Math.abs(enc - prev_enc) < 5 && off != prev_off) break;
                prev_enc = enc;
                prev_off = off;
            }
            else if (off < 0)
            {
                speed = origSpeed * (off / 200);
                if (speed < -0.05) drivetrain.drive(speed, 0, 0);
            }
            else if (off < 300)
            {
                speed = origSpeed * (off / 300);
                if (speed > 0.1) drivetrain.drive(speed, 0, 0);
                else drivetrain.drive(0.06, 0, 0);
            }
            Thread.sleep(1);
        }
        drivetrain.stop();
    }
    
    @Override
    public void run() throws InterruptedException
    {
        double dist = 200;
        double speed = .3;
        
        robot = Robot.instance();
        drivetrain = robot.drivetrain;
        long start = System.currentTimeMillis();
        moveToRange(dist, speed, RIGHT_RANGE);
        long elapsed = System.currentTimeMillis() - start;
        while (opModeIsActive())
        {
            telemetry.addData("Range", robot.rightRange.getDistance());
            telemetry.addData("Elapsed time (ms)", elapsed);
            telemetry.update();
            Thread.sleep(200);
        }
    }
}
