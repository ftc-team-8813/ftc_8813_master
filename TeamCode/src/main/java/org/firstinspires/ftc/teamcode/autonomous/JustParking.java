package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Just Parking")
public class JustParking extends BaseAutonomous {
    @Override
    public void run() throws InterruptedException {
        Robot robot = Robot.instance();
        Thread.sleep(15000);

        robot.drivetrain.move(0.4, 0, 0, tickToInches(25));
        robot.drivetrain.stop();
    }

    public int tickToInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);
        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}
