package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskReadSensor;

/**
 * An example test autonomous OpMode. Test OpModes should perhaps be @Disabled at competitions
 * to declutter?
 */
@Autonomous(name = "Test Autonomous", group="test")
@SuppressWarnings("unused")
public class TestAutonomous extends BaseAutonomous {
    @Override
    public void run() throws InterruptedException {
        //Put tasks here, like this:
        tasks.add(new TaskReadSensor(hardwareMap.colorSensor.get("color_sensor")));
        //If you need to run the tasks before continuing, put:
        //runTasks();
    }
}
