package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectJewel;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDoMove;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskExtendSlide;

import java.util.Arrays;

/**
 * Main autonomous program. RedAutonomous and BlueAutonomous are the sub-OpModes that decide whether
 * isRed or isBlue is true.
 */

public abstract class MainAutonomous extends BaseAutonomous {

    public abstract boolean isBlue();

    private TaskClassifyPictograph finder;
    @Override
    public void initialize() {
        finder = new TaskClassifyPictograph();
    }

    @Override
    public void run() throws InterruptedException {
        tasks.add(finder);
        tasks.add(new TaskExtendSlide(hardwareMap.dcMotor.get("extend")));
        tasks.add(new TaskDoMove("to_jewels.dat"));
        TaskDetectJewel jd = new TaskDetectJewel(hardwareMap.colorSensor.get("color"));
        tasks.add(jd);
        runTasks();
        if (jd.getReading() == TaskDetectJewel.RED) {
            tasks.add(new TaskDoMove("jewel_red.dat"));
        } else if (jd.getReading() == TaskDetectJewel.BLUE) {
            tasks.add(new TaskDoMove("jewel_blue.dat"));
        } else {
            telemetry.addData("Color detected was not valid", Arrays.toString(jd.getDetectedColor()));
            telemetry.update();
        }
        tasks.add(new TaskDoMove("place_block.dat"));
    }
}

@Autonomous(name = "Red Autonomous")
class RedAutonomous extends MainAutonomous {
    @Override
    public boolean isBlue() {
        return false;
    }
}

@Autonomous(name = "Blue Autonomous")
class BlueAutonomous extends MainAutonomous {
    @Override
    public boolean isBlue() {
        return true;
    }
}
