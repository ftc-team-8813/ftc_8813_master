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
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
        robot.foundationhook.moveHookFullDown();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(0.5, 0, 0, tickToInches(17));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0, 0.3, 0, tickToInches(19));
        robot.drivetrain.stop();

        robot.drivetrain.oldMove(0.2, 0, 0, tickToInches(6));
        robot.drivetrain.stop();

        robot.foundationhook.moveHookDown();
        Thread.sleep(2000);

        robot.drivetrain.oldMove(-0.2, 0, 0, tickToInches(20));
        robot.drivetrain.stop();

        robot.drivetrain.setAngleInfluence(.4);
        robot.drivetrain.setTargetAngle(-95);
        Thread.sleep(3000);
        robot.drivetrain.setAngleInfluence(0);
        Thread.sleep(40);

        robot.foundationhook.moveHookUp();

        robot.drivetrain.oldMove(.2,0,0, tickToInches(15));

        robot.drivetrain.oldMove(-.2,0,0, tickToInches(8));

        robot.intakelinkage.moveLinkage(OUT, OUT);

        robot.drivetrain.oldMove(0,.3,0, tickToInches(10));

        Thread.sleep(6000);

        //robot.drivetrain.oldMove(-.3, 0, 0, tickToInches(34));

        robot.drivetrain.oldMove(0,.2,0, tickToInches(10));

        robot.drivetrain.oldMove(-.3,0,0, tickToInches(30));//WALL PARK

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