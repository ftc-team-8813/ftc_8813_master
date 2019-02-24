package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot Autonomous (Other Crater)")
public class Crater2DepotAuto extends DepotAutonomous
{
    @Override
    public void initialize() throws InterruptedException
    {
        OTHER_CRATER = true;
        super.initialize();
    }
}
