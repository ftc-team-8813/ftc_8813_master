package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;

public class MotorArm {
    public DcMotor motorArm;
    public Switch backLimit;

    public MotorArm(DcMotor motorArm, Switch backLimit){
        this.motorArm = motorArm;
        this.backLimit = backLimit;
    }

    public void moveArm(double power){
        if (backLimit.pressed() && power<0){
            motorArm.setPower(0);
        }else{
            motorArm.setPower(power);
        }
    }
}
