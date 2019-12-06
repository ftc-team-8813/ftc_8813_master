package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;


@Autonomous(name="DummyAutoFoundationBlueTriangle")
public class DummyAutoFoundationBlueTriangle extends BaseAutonomous
{
    public void initialize(){
        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.foundationhook.moveHookUp();
        robot.intakelinkage.moveLinkageIn();
        telemetry.addData("FwdEncDegrees", robot.drivetrain.getfrwEnc().getRotations());
        telemetry.update();

        robot.drivetrain.oldMove(-0.3, 0, 0, tickToInches(30));
        robot.drivetrain.stop();
        telemetry.update();

        robot.drivetrain.oldMove(0, 0.3, 0, tickToInches(14));
        robot.drivetrain.stop();
        telemetry.update();

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.oldMove(0.4, 0, 0, tickToInches(30));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(0, -0.6, 0, tickToInches(40));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, 0, 0.3, tickToInches(20));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, 0.2, 0, tickToInches(4));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0.4, 0, 0, tickToInches(9));
    }

    public int tickToInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);
        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}
