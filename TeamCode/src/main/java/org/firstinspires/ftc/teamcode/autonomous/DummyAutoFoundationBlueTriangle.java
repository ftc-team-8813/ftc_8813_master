package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;

import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.OUT;


@Autonomous(name="FoundationBlueTriangle")
public class DummyAutoFoundationBlueTriangle extends BaseAutonomous
{
    public void initialize(){
        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
        robot.foundationhook.moveHookUp();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();

        robot.drivetrain.oldMove(-0.5, 0, 0, tickToInches(17));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, 0.3, 0, tickToInches(23));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(-0.2, 0, 0, tickToInches(6));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.oldMove(0.2, 0, 0, tickToInches(31));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(0, -0.6, 0, tickToInches(28));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(-0.3, 0, 0, tickToInches(15));

        robot.drivetrain.oldMove(0, 0.4, 0, tickToInches(30));

        robot.intakelinkage.moveLinkage(OUT, OUT);
        
        //robot.drivetrain.oldMove(.3, 0, 0, tickToInches(27));

        robot.drivetrain.oldMove(0, -0.4, 0, tickToInches(27));

        robot.drivetrain.oldMove(0,0,0.2, tickToInches(2));
        
        robot.drivetrain.oldMove(0, 0.2, 0, tickToInches(16)); //WALL PARK

        //robot.drivetrain.oldMove(0,-0.3,0,tickToInches(4)); //SKYBRIDGE PARK

        /*robot.drivetrain.move(-0.5, 0, 0, tickToInches(25));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, 0.3, 0, tickToInches(21));
        robot.drivetrain.stop();

        robot.drivetrain.move(-0.2, 0, 0, tickToInches(2));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.move(0.25, 0, 0, tickToInches(32.5));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookUp();
        Thread.sleep(2000);

        robot.drivetrain.oldMove(0, 0.6, 0, tickToInches(30));
        robot.drivetrain.stop();

        robot.drivetrain.move(-0.3, 0, 0, tickToInches(16));

        robot.drivetrain.oldMove(0, 0.4, 0, tickToInches(18));

        robot.drivetrain.move(0, -0.3, 0, tickToInches(25));*/
    }

    public int tickToInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);
        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}