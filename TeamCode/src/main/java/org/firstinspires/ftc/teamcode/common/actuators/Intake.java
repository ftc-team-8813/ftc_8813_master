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
}
