package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.Webcam;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.MED;
import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.OUT;

@Autonomous(name="Stones")
public class StoneAuto extends BaseAutonomous
{
    private Robot robot;
    private WebcamStream stream;
    private SkystoneDetector detector;
    private Drivetrain drivetrain;
    
    private Logger log = new Logger("Stone Autonomous");
    
    private static final int LEFT_RANGE = 1;
    private static final int RIGHT_RANGE = 2;
    
    @Override
    public void initialize() throws InterruptedException
    {
        Robot.instance().intakelinkage.moveLinkageIn();
        Robot.instance().claw.openClaw();
        CameraStream rawStream = getCameraStream();
        if (rawStream instanceof WebcamStream)
        {
            stream = (WebcamStream)rawStream;
        }
        else
        {
            throw new IllegalStateException("Only webcam is supported. Is it plugged in?");
        }
        // ExposureControl control = stream.getInternalCamera().getControl(ExposureControl.class);
        // control.setMode(ExposureControl.Mode.Manual);
        Robot.instance().imu.setImmediateStart(true);
        Robot.instance().imu.initialize();
    }
    
    private void pickBlock(int turn, int strafe_dist, int fwd_dist, int back_dist) throws InterruptedException
    {
        int off = 0;
        if (detector.found())
        {
            off = detector.getArea().x + detector.getArea().width/2 - 320;
        }
        drivetrain.move(0.3, 0, 0, 50);
        drivetrain.move(0.3, 0, 0, 150 - off);
        Thread.sleep(100);

        // drivetrain.move(0.4, 0, 0, strafe_dist);
        /*
        turnToAngle(-turn, 0.4);
        Thread.sleep(100);
        robot.intake.collectStone(0.4);
        drivetrain.move(0.4, 0, 0, fwd_dist);
        Thread.sleep(400);
         */
        robot.intakelinkage.moveLinkage(OUT, MED);
        Thread.sleep(350);
        robot.intake.collectStone(0.4);
        drivetrain.drive(0.3, 0, 0);
        Thread.sleep(450);
        robot.intakelinkage.moveLinkage(OUT, OUT);
        // curveTurn(0.2, 800);
        Thread.sleep(400);
        drivetrain.stop();
        robot.intake.stopIntake();
        turnToAngle(0, 0.3);
        Thread.sleep(100);
        drivetrain.move(0.5, 0, 0, -back_dist  - 8);
        // drivetrain.move(0, 0, 0.6, -((int)robot.imu.getHeading()) + 50);
    }
    
    private void pickBlock2(int turn, int strafe_dist, int fwd_dist, int back_dist) throws InterruptedException
    {
        int off = 0;
        if (detector.found())
        {
            off = detector.getArea().x + detector.getArea().width/2 - 320;
        }
        
        drivetrain.move(0, 0.3, 0, off - 75 - strafe_dist);
        // drivetrain.move(0.4, 0, 0, strafe_dist);
        /*
        turnToAngle(-turn, 0.4);
        Thread.sleep(100);
        robot.intake.collectStone(0.4);
        drivetrain.move(0.4, 0, 0, fwd_dist);
        Thread.sleep(400);
         */
        robot.intakelinkage.moveLinkage(OUT, MED);
        Thread.sleep(350);
        robot.intake.collectStone(0.4);
        drivetrain.drive(0.3, 0, -0.2);
        Thread.sleep(400);
        robot.intakelinkage.moveLinkage(OUT, OUT);
        // curveTurn(0.2, 800);
        Thread.sleep(400);
        drivetrain.stop();
        robot.intake.stopIntake();
        turnToAngle(0, 0.3);
        Thread.sleep(100);
        drivetrain.move(0.5, 0, 0, -back_dist  - 2);

    }
    
    private int senseBlock() throws InterruptedException
    {
        detector.enable();
        stream.getInternalCamera().getControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        int startPos = drivetrain.rightBack.getCurrentPosition();
        drivetrain.drive(0, 0.3, 0);
        final double SPD_NORMAL = 0.3;
        final double SPD_SLOW   = 0.12;
        final double SPD_EXSLOW = 0.05;
        double speed = 0;
    
        while (drivetrain.rightBack.getCurrentPosition() - startPos < 1800)
        {
            if (detector.found())
            {
                if (detector.getArea().width > 240 && detector.getCenter().y < 320)
                {
                    drivetrain.drive(SPD_SLOW, 0, 0);
                }
                else if (detector.getArea().width > 350)
                {
                    break;
                }
                else if (detector.getArea().width > 120)
                {
                    speed -= (speed - SPD_SLOW) * 0.4;
                    drivetrain.drive(0, speed, 0);
                }
                else
                {
                    speed -= (speed - SPD_NORMAL) * 0.4;
                    drivetrain.drive(0, speed, 0);
                }
            }
            else
            {
                drivetrain.drive(0, SPD_NORMAL, 0);
            }
            Thread.sleep(1);
        }
    
        drivetrain.stop();
        detector.disable();
        Thread.sleep(100);
        return drivetrain.rightBack.getCurrentPosition() - startPos;
    }
    
    private void moveToRange(double dist, double speed, int sensor) throws InterruptedException
    {
        double origSpeed = speed;
        drivetrain.drive(speed, 0, 0);
        RangeSensor sens;
        if (sensor == LEFT_RANGE) sens = robot.centerRange;
        else sens = robot.centerRange;
        
        double val = sens.getDistance();
        if (val == 65535)
        {
            log.f("Distance sensor not plugged in");
            telemetry.addData("FAIL", "The robot is blind--the distance sensor isn't plugged in!");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        else if (val >= 8190)
        {
            log.f("Distance sensor reads 8192");
            telemetry.addData("FAIL", "The robot is blind--the distance sensor can't see anything");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        
        // dist += 110; // Center range sensor offset
        int prev_enc = 0;
        double prev_off = Double.POSITIVE_INFINITY;
        int count = 0;
        long lastIter = System.currentTimeMillis();
        while (true)
        {
            double range = sens.getDistance();
            double off = (range - dist) * Math.signum(origSpeed);
            if (off <= 0 && off > -50)
            {
                drivetrain.stop();
                int enc = drivetrain.rightBack.getCurrentPosition();
                // log.v("Overshoot correction: overshoot=%.0f, speed=%.3f, enc delta=%d", -off, speed, enc - prev_enc);
                if (Math.abs(enc - prev_enc) < 5 && off != prev_off) break;
                prev_enc = enc;
                prev_off = off;
            }
            else if (off < 0)
            {
                speed = origSpeed * (off / 150);
                if (speed < -0.05) drivetrain.drive(speed, 0, 0);
            }
            else if (off < 300)
            {
                speed = origSpeed * (off / 300);
                if (speed > 0.1) drivetrain.drive(speed, 0, 0);
                else drivetrain.drive(0.1, 0, 0);
            }
            count++;
            if (System.currentTimeMillis() - lastIter > 1000)
            {
                lastIter = System.currentTimeMillis();
                log.d("Loops per second: %d", count);
                count = 0;
            }
            Thread.sleep(1);
        }
        drivetrain.stop();
        log.i("Reached a distance of %.0f mm (%.0f error from %.0f)",
                sens.getDistance(), Math.abs(dist - sens.getDistance()), dist);
    }
    
    private void turnToAngle(double angle, double speed) throws InterruptedException
    {
        IMU imu = robot.imu;
        double prev = imu.getHeading() - angle;
        while (true)
        {
            double error = imu.getHeading() - angle;
            double delta = error - prev;
            prev = error;
            if (Math.abs(error) < 2)
            {
                drivetrain.stop();
                if (delta == 0) break;
            }
            else
            {
                drivetrain.drive(0, 0, -(error / 35) * speed + 0.05 * Math.signum(-error));
            }
            Thread.sleep(1);
        }
        drivetrain.stop();
    }
    
    private void curveTurn(double speed, long msecs) throws InterruptedException
    {
        drivetrain.drive(speed*1.4, 0, -speed*0.9);
        Thread.sleep(msecs);
        drivetrain.stop();
    }
    
    @Override
    public void run() throws InterruptedException
    {
        /////////////////
        // Initialization
        // Robot
        robot = Robot.instance();
        robot.intakelinkage.moveLinkageOut();
        // Drivetrain
        drivetrain = robot.drivetrain;
        // Vision
        detector = new SkystoneDetector();
        detector.disable(); // Disable until we start detecting
        stream.addListener(detector);
        stream.addModifier(detector);
        stream.addListener(new Vlogger("videos/" + Utils.getTimestamp() + ".avi",  640, 480, 10), 1001);
        // GlobalDataLogger.instance().addChannel("Skystone Width", () -> "" + (detector.found() ? detector.getArea().width : 0));
        
        // Initial forward
        // moveToRange(280, 0.5, RIGHT_RANGE);
        drivetrain.move(0.4, 0, 0, 425);
        Thread.sleep(100);
        drivetrain.move(0, 0.4, 0, 375);
        Thread.sleep(100);
        
        // Sense block
        int senseDist = senseBlock();
        
        // turnToAngle(0, 0.35);
        pickBlock2(25, 0, 350, 175);
        // drivetrain.move(0.3, 0, 0, 50);
        
        // drivetrain.move(0, 0, 0.2, -(int)robot.imu.getHeading()); // Another fine correction
        Thread.sleep(100);
        drivetrain.move(0, 1, 0, -(int)(senseDist + 1700));
        Thread.sleep(150);
        // drivetrain.move(0, 0, 0.6, -((int)robot.imu.getHeading()) + 50);
        robot.intake.collectStone(-0.4);
        Thread.sleep(400);
        robot.intake.stopIntake();
        turnToAngle(0, 0.4);
        //drivetrain.move(0.4, 0, 0, -25);
        drivetrain.move(0, 1, 0, 750);
        // moveToRange(280, 0.5, RIGHT_RANGE);
        // drivetrain.move(0, 0, 0.1, -(int)robot.imu.getHeading());
        
        senseDist = senseBlock();
        // drivetrain.move(0, 0.5, 0, 25);
        turnToAngle(0, 0.4);
        
        // drivetrain.move(0.6, 0, 0, -1);
        pickBlock(25, 0, 350, 400);
        
        robot.intake.collectStone(0.5);
        Thread.sleep(500);
        robot.intake.stopIntake();
        
        drivetrain.move(0, 1, 0, -(int)(senseDist + 600));
        robot.intake.collectStone(-0.5);
        Thread.sleep(600);
        robot.intake.stopIntake();
        drivetrain.move(0, 1, 0, 300);
    }
    
    @Override
    public void finish() throws InterruptedException
    {
    
    }
}
