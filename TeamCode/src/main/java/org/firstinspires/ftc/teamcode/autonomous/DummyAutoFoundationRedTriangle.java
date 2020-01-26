package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;

import static org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage.OUT;


@Autonomous(name="FoundationRedTriangle")
public class DummyAutoFoundationRedTriangle extends BaseAutonomous
{
    public void initialize(){
        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
        robot.foundationhook.moveHookUp();
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();

        robot.drivetrain.oldMove(-0.5, 0, 0, tickToInches(20));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, -0.3, 0, tickToInches(27));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(-0.2, 0, 0, tickToInches(5));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.oldMove(0.2, 0, 0, tickToInches(31));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(0, 0.6, 0, tickToInches(27));
        robot.drivetrain.stop();
        
        Thread.sleep(10000);

        robot.drivetrain.oldMove(-0.3, 0, 0, tickToInches(10));

        robot.drivetrain.oldMove(0, -0.4, 0, tickToInches(15));

        robot.drivetrain.oldMove(0, 0.4, 0, tickToInches(28));

        robot.drivetrain.oldMove(.2,0,0,4);
        
        robot.intakelinkage.moveLinkage(OUT, OUT);

        robot.drivetrain.oldMove(0,0,-0.2, tickToInches(2));

        //robot.drivetrain.oldMove(0,-0.2,0,tickToInches(16)); //WALL PARK

        robot.drivetrain.oldMove(0, 0.4, 0, tickToInches(6)); //SKYBRIDGE PARK

        /*obot.foundationhook.moveHookDown();
        Thread.sleep(1000);
        robot.foundationhook.moveHookUp();


        robot.drivetrain.move(-0.5, 0, 0, tickToInches(25));

        robot.drivetrain.oldMove(0, -0.3, 0, tickToInches(28));

        robot.drivetrain.move(-0.2, 0, 0, tickToInches(4));

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.move(0.25, 0, 0, tickToInches(35));
        Thread.sleep(1);

        robot.drivetrain.oldMove( 0, 0.1, 0, tickToInches(3));

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(0, 0.6, 0, tickToInches(30));
        Thread.sleep(1000);

        robot.drivetrain.move(-0.3, 0, 0, tickToInches(16));

        robot.drivetrain.oldMove(0, -0.4, 0, tickToInches(18));

        robot.drivetrain.move(0, 0.3, 0, tickToInches(25));*/
    }

    public int tickToInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);
        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}