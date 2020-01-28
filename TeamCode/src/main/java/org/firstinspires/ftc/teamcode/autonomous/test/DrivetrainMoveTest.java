package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.Arrays;

@Autonomous(name="Encoder Move Test")
public class DrivetrainMoveTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException {
        Drivetrain drivetrain = Robot.instance().drivetrain;
        Logger log = new Logger("Move Test");
        drivetrain.enableAngleCorrection();
        Robot.instance().fwdEnc.resetEncoder();
        Robot.instance().strafeEnc.resetEncoder();
    
        GlobalThreadPool.instance().start(() ->
        {
           while (true)
           {
               telemetry.addData("Fwd encoder", Robot.instance().fwdEnc.getPosition());
               telemetry.addData("Strafe encoder", Robot.instance().strafeEnc.getPosition());
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
        
        Robot robot = Robot.instance();
        
        double start = drivetrain.rightBack.getCurrentPosition();
        double startOdo = robot.fwdEnc.getPosition();
        drivetrain.move(0.3, 0, 0, 100);
        Thread.sleep(1000);
        log.i("Distance moved forward: %.0f/%.2f",
                drivetrain.rightBack.getCurrentPosition() - start,
                robot.fwdEnc.getPosition() - startOdo);
        
        start = drivetrain.rightBack.getCurrentPosition();
        startOdo = robot.strafeEnc.getPosition();
        drivetrain.move(0, 0.3, 0, 100);
        Thread.sleep(1000);
        log.i("Distance strafed: %.0f/%.2f",
                drivetrain.rightBack.getCurrentPosition() - start,
                robot.strafeEnc.getPosition());
        drivetrain.stop();
        
        /*
        while (opModeIsActive())
        {
            telemetry.addData("Done", Robot.instance().fwdEnc.getPosition() - start);
            telemetry.addData("Absolute angle", Robot.instance().fwdEnc.getPosition());
            telemetry.update();
            Thread.sleep(100);
        };
         */
    }
}
