package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;

@Autonomous(name="Encoder Move Test")
public class DrivetrainMoveTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException {
        Drivetrain drivetrain = Robot.instance().drivetrain;
        drivetrain.enableAngleCorrection();
        
        double start = Robot.instance().strafeEnc.getAbsoluteAngle();
        drivetrain.move(0, 0.4, 0, 100);
        Thread.sleep(500);
        while (opModeIsActive())
        {
            telemetry.addData("Done", Robot.instance().strafeEnc.getAbsoluteAngle() - start);
            telemetry.addData("Absolute angle", Robot.instance().strafeEnc.getAbsoluteAngle());
            telemetry.update();
            Thread.sleep(100);
        }
    }
}
