package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Autonomous")
public class BlueAutonomous extends MainAutonomous {
    @Override
    public boolean isBlue() {
        return true;
    }
}
