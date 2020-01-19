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
        
        double[] distancesF = new double[10];
        double start = Robot.instance().fwdEnc.getPosition();
        
        for (int i = 0; i < 3; i++)
        {
            drivetrain.move(0.4, 0, 0, 100);
            Thread.sleep(500);
            distancesF[i] = Robot.instance().fwdEnc.getPosition() - start;
            drivetrain.move(0.4, 0, 0, -100);
            Thread.sleep(500);
        }
        
        
        double[] distancesS = new double[10];
        start = Robot.instance().strafeEnc.getPosition();
        for (int i = 0; i < 3; i++)
        {
            drivetrain.move(0, 0.4, 0, 100);
            Thread.sleep(500);
            distancesS[i] = Robot.instance().strafeEnc.getPosition() - start;
            drivetrain.move(0, 0.4, 0, -100);
            Thread.sleep(500);
        }
        
        log.i("Forward distances: %s", Arrays.toString(distancesF));
        log.i("Strafe distances: %s", Arrays.toString(distancesS));
        
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
