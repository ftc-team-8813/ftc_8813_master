package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Switch Tester")
public class SwitchTester extends OpMode
{
    private List<Switch> switches;

    @Override
    public void init()
    {
        String[] digitalChannels = Utils.allDeviceNames(hardwareMap.digitalChannel);
        switches = new ArrayList<>();
        for (String s : digitalChannels)
        {
            switches.add(new Switch(hardwareMap.digitalChannel.get(s)));
        }
    }

    @Override
    public void loop()
    {

    }
}
