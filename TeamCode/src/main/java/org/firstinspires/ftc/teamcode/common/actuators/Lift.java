package org.firstinspires.ftc.teamcode.common.actuators;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;

import java.io.File;

public class Lift {
    public PIDMotor slidemotor;
    private Switch bottomswitch;
    DataStorage topswitch = new DataStorage(new File(Config.storageDir + "liftencoderpos.txt"));

    public Lift(PIDMotor slidemotor, Switch bottomswitch){
        this.slidemotor = slidemotor;
        this.bottomswitch = bottomswitch;
    }

    public void raiseLift(double power){
        if (power < 0 && !bottomswitch.pressed()) {
            if (slidemotor.isHolding()) {
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power * 0.25);
        } else if (power > 0 && slidemotor.getCurrentPosition() >= topswitch.getInt("Highest Position", 0) ){
            if (slidemotor.isHolding()){
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power);
        } else {
            if (!slidemotor.isHolding()) {
                slidemotor.setPower(0.5);
                slidemotor.hold(slidemotor.getCurrentPosition());
            }
        }
    }

    public double getCurrentPos(){
        return slidemotor.getCurrentPosition();
    }


}
