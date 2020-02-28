package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Just Parking")
public class JustParking extends BaseAutonomous {
    public void initialize()
    {
        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
        robot.foundationhook.moveHookFullDown();
        robot.newarm.resetArm();
    }
    @Override
    public void run() throws InterruptedException {
        Robot robot = Robot.instance();
//        Thread.sleep(15000);

        robot.intakelinkage.moveLinkageOut();
        robot.drivetrain.move(0.4, 0, 0, 100);
        robot.drivetrain.stop();

        robot.claw.closeClaw();

        robot.slide.raiseLiftAsync(0.9, 800);
    }

    public int tickToInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);

        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}
