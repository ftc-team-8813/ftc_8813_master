package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous")
public class RedAutonomous extends MainAutonomous {
    @Override
    public boolean isBlue() {
        return false;
    }
}