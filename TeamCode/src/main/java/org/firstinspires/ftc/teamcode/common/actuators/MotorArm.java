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
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveArm(double power){
        // motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (power<0 && backLimit.pressed()){
            motorArm.setPower(0);
            if (motorArm.getCurrentPosition() != 0)
            {
                motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else if (power > 0 && motorArm.getCurrentPosition()>930){
            motorArm.setPower(0);
        }else{
            motorArm.setPower(power);
        }
    }

    public void moveArmEnc(double power, int pos){
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setTargetPosition(pos + motorArm.getCurrentPosition());
        motorArm.setPower(power);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void moveArmTo(double power, int pos) throws InterruptedException
    {
        motorArm.setTargetPosition(pos);
        motorArm.setPower(power);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(motorArm.getCurrentPosition() - pos) > 15)
        {
            Thread.sleep(5);
        }
        motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetArm(){
        while (!backLimit.pressed()){
            motorArm.setPower(-0.3);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
