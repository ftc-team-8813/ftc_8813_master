package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.motor_control.AccelMotor;
import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.Objects;

@Autonomous(name="Acceleration Control Test")
public class AccelMotorTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Logger log = new Logger("AccelMotorTest");
        PIDMotor motor = Robot.instance().drivetrain.leftFront;
        GlobalThreadPool.instance().start(() ->
        {
            while (true)
            {
                telemetry.addData("Position", motor.getCurrentPosition());
                telemetry.update();
            }
        });
        AccelMotor accelMotor = (AccelMotor)motor.getMotor();
        accelMotor.setPower(0.5);
        Thread.sleep(2000);
        accelMotor.setPower(0);
        Thread.sleep(2000);
        
        motor.setPower(0.5);
        log.d("Position: %d", motor.getCurrentPosition());
        motor.startRunToPosition(0);
    }
}
