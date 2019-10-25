package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.concurrent.Future;

@TeleOp(name="Swerve Drive")
public class SwerveDrive extends BaseTeleOp
{
    private Future<?> monitor;
    
    @Override
    public void init()
    {
        super.init();
        
        // CALIBRATION
        robot.frontWheel.resetEncoders();
        robot.backWheel.resetEncoders();
        // Initialize calibration
        robot.frontWheel.calibrate(robot.frontHall);
        robot.backWheel.calibrate(robot.backHall);
        telemetry.addData("Front calibration", "In progress");
        telemetry.addData("Back calibration", "In progress");
        // Start a monitor thread
        monitor = GlobalThreadPool.instance().start(() ->
        {
            while (robot.frontWheel.calibrating() || robot.backWheel.calibrating())
            {
                telemetry.addData("Front calibration", robot.frontWheel.calibrating() ? "In progress" : "Done");
                telemetry.addData("Back calibration", robot.backWheel.calibrating() ? "In progress" : "Done");
                try
                {
                    Thread.sleep(100); // This isn't timing-critical
                }
                catch (InterruptedException e)
                {
                    break;
                }
                telemetry.update();
            }
            telemetry.addData("Calibration", "finished");
            telemetry.update();
            robot.backWheel.copy(robot.frontWheel); // Experimental but works :D
        });
    }
    
    @Override
    public void loop()
    {
        // Don't drive until we're done calibrating
        if (!monitor.isDone())
        {
            return;
        }
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_y;
        robot.frontWheel.drive(turn + forward, turn - forward);
        // And the back wheel will follow
    }
    
    @Override
    public void stop()
    {
        super.stop();
        if (monitor != null && !monitor.isDone())
        {
            monitor.cancel(true);
        }
    }
}
