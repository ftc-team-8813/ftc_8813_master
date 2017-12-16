package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.util.DcMotorUtil;

/**
 * Created by aidan on 12/14/17.
 */
@Autonomous(name="Encoder Test")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorUtil.moveToPosition(
                hardwareMap.dcMotor.get("base"),
                DcMotorUtil.degreesToEncoders(180, 60),
                0.5);
    }
}
