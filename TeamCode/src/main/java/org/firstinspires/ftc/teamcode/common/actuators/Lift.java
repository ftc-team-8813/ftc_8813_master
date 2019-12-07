package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;

import java.io.File;

public class Lift {
    public PIDMotor slidemotor;
    private Switch bottomswitch;
    DataStorage topswitch;
    private int toplimit;

    public Lift(PIDMotor slidemotor, Switch bottomswitch){
        this.slidemotor = slidemotor;
        this.bottomswitch = bottomswitch;
        topswitch = new DataStorage(new File(Config.storageDir + "liftencoderpos.txt"));
        toplimit = topswitch.getInt("Highest Position", 0);
        slidemotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void raiseLift(double power){
        if (power < 0 && !bottomswitch.pressed()) {
            if (slidemotor.isHolding()) {
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power * 0.125);
        } else if (power > 0 && slidemotor.getCurrentPosition() <= toplimit){
            if (slidemotor.isHolding()){
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power);
        } else {
            if (!slidemotor.isHolding()) {
                slidemotor.getMotor().setPower(0);
                slidemotor.setPower(0.5);
                slidemotor.hold(-slidemotor.getCurrentPosition());
            }
        }
    }

    public void raiseLiftEnc(int dist) throws InterruptedException {
        slidemotor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidemotor.runToPosition(dist, true);
    }

    public void resetLift() throws InterruptedException {
        slidemotor.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!bottomswitch.pressed()){
            slidemotor.setPower(0.1);
            Thread.sleep(100);
        }
        slidemotor.setPower(0);
    }

    public double getCurrentPos(){
        return slidemotor.getCurrentPosition();
    }

    public double getTopLimit(){
        return toplimit;
    }


}
