package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.common.util.Persistent;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.MED;
import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.OUT;

/*
    IDEAS BIN
    
    Curve turn to pick up block
    Enable field centric drive

 */


@Autonomous(name="Blue Stone-Stone Auto")
public class StoneAuto extends BaseAutonomous
{
    private Robot robot;
    private WebcamStream stream;
    private SkystoneDetector detector;
    private Drivetrain drivetrain;
    
    private Logger log = new Logger("Stone Autonomous");
    
    private static final int LEFT_RANGE = 1;
    private static final int RIGHT_RANGE = 2;
    
    private Profiler profiler;
    
    @Override
    public void initialize() throws InterruptedException
    {
        Robot.instance().centerRange.disable();
        Robot.instance().foundationhook.moveHookFullDown();
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
        
        Robot.instance().newarm.resetArm();
        Robot.instance().imu.initialize();
        Robot.instance().imu.waitForInit(telemetry);
        Robot.instance().imu.start();
        Persistent.put("imu", Robot.instance().imu);
        
        profiler = new Profiler();
        
        Robot.instance().odometry.reset();
        
        /*
        GlobalThreadPool.instance().start(() ->
        {
            while (true)
            {
                try
                {
                    telemetry.addData("IMU heading", robot.imu.getHeading());
                    log.d("IMU heading", robot.imu.getHeading());
                    telemetry.update();
                    Thread.sleep(1000);
                }
                catch (InterruptedException e)
                {
                    log.d("Logger interrupted!");
                    break;
                }
            }
        });
         */
    }
    
    private void pickBlock(int turn, int strafe_dist, int fwd_dist, int back_dist) throws InterruptedException
    {
        int off = 0;
        if (detector.found())
        {
            off = detector.getArea().x + detector.getArea().width/2 - 320;
        }
        // drivetrain.move(0.3, 0, 0, 5);
        //
        drivetrain.moveTimeout(0, 0.52, 0, strafe_dist, 300);
        Thread.sleep(100);

        // drivetrain.move(0.4, 0, 0, strafe_dist);
        /*  
        turnToAngle(-turn, 0.4);
        Thread.sleep(100);
        robot.intake.collectStone(0.4);
        drivetrain.move(0.4, 0, 0, fwd_dist);
        Thread.sleep(400);
         */
        robot.intakelinkage.moveLinkage(MED, OUT);
        Thread.sleep(350);
        robot.intake.collectStone(0.55);
        drivetrain.drive(0.4, 0, 0);
        Thread.sleep(750);
        robot.intakelinkage.moveLinkage(OUT, OUT);
        //robot.intake.collectStone(0.55);
        // curveTurn(0.2, 800);
        Thread.sleep(400);
        drivetrain.stop();
        Thread.sleep(600);
        robot.intake.stopIntake();
        // turnToAngle(0, 0.3);
        Thread.sleep(100);
        drivetrain.move(0.42, 0, 0, -back_dist  - 7);
        robot.intake.collectStone(.55);
        Thread.sleep(400);
        robot.intake.stopIntake();
        robot.claw.closeClaw();
        //  Thread.sleep(400);
        //  robot.intake.stopIntake();
        //  robot.claw.closeClaw();
        // drivetrain.move(0, 0, 0.6, -((int)robot.imu.getHeading()) + 50);
    }
    
    private void pickBlock2(int turn, int strafe_dist, int fwd_dist, int back_dist) throws InterruptedException
    {
        int off = 0;
        if (detector.found())
        {
            off = detector.getArea().x + detector.getArea().width/2 - 320;
        }
        
        drivetrain.move(0, 0.52, 0, off - 75 - strafe_dist);
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
        robot.intake.collectStone(0.42);
        drivetrain.drive(0.32, 0, -0.2);
        Thread.sleep(400);
        robot.intakelinkage.moveLinkage(OUT, OUT);
        // curveTurn(0.2, 800);
        Thread.sleep(400);
        drivetrain.stop();
        robot.intake.stopIntake();
        turnToAngle(0, 0.32);
        Thread.sleep(100);
        drivetrain.move(0.25, 0, 0, -back_dist);
        Thread.sleep(500);

    }
    
    private int senseBlock2(double distance) throws InterruptedException
    {
        double fwdStart = robot.odometry.getForwardDistance();
        detector.enable();
        while (opModeIsActive())
        {
            if (detector.found())
            {
                double xError = detector.getCenter().x - 320;
                drivetrain.drive(0.3, xError * 0.0015, 0);
            }
            else
            {
                drivetrain.stop();
            }
            if (robot.odometry.getForwardDistance() - fwdStart > distance) break;
            Thread.sleep(5);
        }
        drivetrain.stop();
        detector.disable();
        Thread.sleep(100);
        double[] delta = drivetrain.updateTarget();
        return (int)delta[1];
    }

    private int senseBlock(int direction) throws InterruptedException
    {
        detector.enable();
        stream.getInternalCamera().getControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        int startPos = drivetrain.rightBack.getCurrentPosition();
        drivetrain.drive(0, 0.32, 0);
        final double SPD_NORMAL = 0.14;
        final double SPD_SLOW   = 0.12;
        final double SPD_EXSLOW = 0.05;
        double speed = 0;

        while (Math.abs(drivetrain.rightBack.getCurrentPosition() - startPos) < 1000)
        {
            if (detector.found())
            {
                if (detector.getArea().width > 240 && detector.getCenter().y < 300)
                {
                    drivetrain.drive(SPD_SLOW, 0, 0);
                }
                else if (detector.getArea().width > 350)
                {
                    break;
                }
                else if (detector.getArea().width > 120)
                {
                    speed = SPD_SLOW * direction;
                    drivetrain.drive(0, speed, 0);
                }
                else
                {
                    speed = SPD_NORMAL * direction;
                    drivetrain.drive(0, speed, 0);
                }
            }
            else
            {
                drivetrain.drive(0, SPD_NORMAL * direction, 0);
            }
            Thread.sleep(1);
        }
    
        drivetrain.stop();
        detector.disable();
        Thread.sleep(100);
        double[] delta = drivetrain.updateTarget();
        return (int)delta[1];
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
        profiler.start("run");
        /////////////////
        // Initialization
        // Robot
        robot = Robot.instance();
        robot.intakelinkage.moveLinkage(MED,MED);
        // Drivetrain
        drivetrain = robot.drivetrain;
        // Vision
        detector = new SkystoneDetector();
        detector.disable(); // Disable until we start detecting
        stream.addListener(detector);
        stream.addModifier(detector);
        stream.addListener(new Vlogger("videos/" + Utils.getTimestamp() + ".avi",  640, 480, 10), 1001);
        drivetrain.enableAngleCorrection();
        drivetrain.enableFieldCentric();
        drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initial forward

        int senseDist = senseBlock2(80);
        Thread.sleep(100);
        drivetrain.moveDiag(7,0.3,19,.9);

        robot.newarm.moveArmEnc(1,800);
        Thread.sleep(800);
        robot.claw.closeClaw();
        Thread.sleep(400);
        robot.intake.collectStone(.65);
        robot.intakelinkage.moveLinkageOut();
        robot.newarm.moveArmEnc(1,-800);
        Thread.sleep(450);
        robot.claw.openClaw();
        Thread.sleep(500);
        robot.intakelinkage.moveLinkageIn();
        Thread.sleep(400);
        robot.intake.stopIntake();
        robot.claw.closeClaw();

        drivetrain.moveDiag(-17,.4,-350 - senseDist,.75);
        drivetrain.stop();

        robot.slide.raiseLiftAsync(.9,800);
        robot.foundationhook.moveHookUp();
        drivetrain.move(.23,0,0,39);
        robot.newarm.moveArmEnc(1,550);
        Thread.sleep(400);
        robot.foundationhook.moveHookDown();
        robot.slide.raiseLift(0,-1);
        robot.claw.openClaw();
        robot.newarm.moveArmEnc(1,-550);

        drivetrain.move(.5,0,0,-50);
        drivetrain.stop();
        drivetrain.setAngleInfluence(0.4);
        drivetrain.setTargetAngle(90);
        drivetrain.drive(-0.3, 0.2, 0);
        Thread.sleep(1500);
        robot.foundationhook.moveHookUp();
        drivetrain.stop();

        drivetrain.updateTarget();
        drivetrain.moveDiag(-15,.05,370 ,.9);
        drivetrain.setAngleInfluence(0.4);
        drivetrain.setTargetAngle(-5);
        drivetrain.drive(0.3, 0.1, 0);
        Thread.sleep(1000);
        drivetrain.stop();

        //drivetrain.move(0.3,.3,0,12);
        senseBlock(1);
        robot.drivetrain.moveDiag(3,.3,17,.8);

        robot.intakelinkage.moveLinkageOut();
        robot.newarm.moveArmEnc(1,950);
        Thread.sleep(800);
        robot.claw.closeClaw();
        Thread.sleep(400);
        robot.newarm.moveArmEnc(1,-1100);
        robot.intake.collectStone(.5);
        Thread.sleep(1000);
        robot.claw.openClaw();
        Thread.sleep(300);
        robot.intakelinkage.moveLinkageIn();
        robot.foundationhook.moveHookDown();
        Thread.sleep(300);
        robot.intake.stopIntake();
        robot.claw.closeClaw();

        drivetrain.updateTarget();
        robot.newarm.moveArmEnc(1,-1100);
        drivetrain.drive(-.2,-.8,0);
        Thread.sleep(2750);
        drivetrain.updateTarget();
        drivetrain.moveDiag(-5,.4,175,.6);
        drivetrain.stop();
        
       /* drivetrain.drive(-.2,.8,0);
        Thread.sleep(2200);
        drivetrain.updateTarget();
        drivetrain.move(.3,0,0,12);

        senseBlock(1);
        Thread.sleep(100);

        drivetrain.move(0.2,.3,0,17);
        robot.intakelinkage.moveLinkageOut();
        robot.newarm.moveArmEnc(.8,725);
        Thread.sleep(800);
        robot.claw.closeClaw();
        Thread.sleep(400);
        robot.intake.collectStone(.5);
        robot.newarm.moveArmEnc(1,-800);
        Thread.sleep(450);
        robot.claw.openClaw();
        Thread.sleep(500);
        robot.intakelinkage.moveLinkageIn();
        Thread.sleep(400);
        robot.intake.stopIntake();
        robot.claw.closeClaw();

        drivetrain.drive(-.1,-.8,0);
        Thread.sleep(2500);
        drivetrain.stop(); */



    }
    
    @Override
    public void finish() throws InterruptedException
    {
    
    }
}
