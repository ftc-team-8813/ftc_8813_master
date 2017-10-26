package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous")
public class RedAutonomous extends BasicAutonomous {
    @Override
    public boolean isBlue() {
        return false;
    }
}
