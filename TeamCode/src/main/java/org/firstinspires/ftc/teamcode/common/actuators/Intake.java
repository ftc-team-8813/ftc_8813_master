package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    public Intake(DcMotor leftIntake, DcMotor rightIntake){
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(double left, double right)
    {
        leftIntake.setPower(left);
        rightIntake.setPower(-right);
    }

    public void collectStone(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(-power);
    }

    public void releaseStone(double power){
        leftIntake.setPower(-power);
        rightIntake.setPower(power);
    }

    public void stopIntake(){
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    
    public double getPower()
    {
        return leftIntake.getPower();
    }
}
