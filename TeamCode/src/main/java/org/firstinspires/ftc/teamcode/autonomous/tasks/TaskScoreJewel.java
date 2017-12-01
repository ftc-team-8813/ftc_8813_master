package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

import static java.lang.Thread.sleep;

/**
 * Created by Joseph Murphy on 12/1/2017.
 */

public class TaskScoreJewel implements Task{
    DcMotor base;
    Servo colorArm;
    boolean isBlue;
    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;
    byte[] color;
    public TaskScoreJewel(int quadrant){
        HardwareMap hardwareMap = BaseAutonomous.instance().hardwareMap;
        base = hardwareMap.dcMotor.get("base");
        colorSensor = hardwareMap.i2cDevice.get("color_sensor");
        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorArm = hardwareMap.servo.get("colorArm");
        if(quadrant == 2 || quadrant == 4){
            isBlue = false;
        }else{
            isBlue = true;
        }
    }
    @Override
    public void runTask() throws InterruptedException {
        boolean isRed;
        if(isBlue){base.setTargetPosition(3600);}
        else{base.setTargetPosition(1);}
        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        base.setPower(.25);
        colorArm.setPosition(.75);
        colorSensorReader.engage();
        color = colorSensorReader.read(0x04, 1);
        if(color[0] < 4){
            isRed = false;
        }else{
            isRed = true;
        }
        if(isBlue) {
            if (isRed) {
                base.setTargetPosition(928490823);
            } else {
                base.setTargetPosition(902180);
            }
        }
        if(isBlue) {
            if (isRed) {
                base.setTargetPosition(928490823);
            } else {
                base.setTargetPosition(902180);
            }
        }
    }
}
