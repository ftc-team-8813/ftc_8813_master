package org.firstinspires.ftc.teamcode.common.actuators;

import android.provider.Settings;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;

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
    
        GlobalDataLogger.instance().addChannel("Lift Position", () -> Integer.toString(this.slidemotor.getCurrentPosition()));
        GlobalDataLogger.instance().addChannel("Lift Target", () -> Integer.toString(this.slidemotor.getTargetPosition()));
        GlobalDataLogger.instance().addChannel("Lift Power", () -> Double.toString(this.slidemotor.getPower()));
        GlobalDataLogger.instance().addChannel("Lift Limit State", () -> this.bottomswitch.pressed() ? "1" : "0");
    }
    
    public void raiseLift(double power)
    {
        if (power <= 0) raiseLift(0.01, (int)Math.floor(power));
        else raiseLift(power, 1);
    }

    public void raiseLift(double power, int direction){
        if (direction < 0 && !bottomswitch.pressed()) {
            if (slidemotor.isHolding()) {
                slidemotor.stopHolding();
                slidemotor.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            slidemotor.getMotor().setPower(power);
        } else if (direction > 0 && slidemotor.getCurrentPosition() <= toplimit){
            if (slidemotor.isHolding()){
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power);
        } else {
            if (!slidemotor.isHolding()) {
                slidemotor.getMotor().setPower(0);
                slidemotor.setPower(0.5);
                slidemotor.hold(slidemotor.getCurrentPosition() + 35);
            }
        }
    }

    public void raiseLiftEnc(double power, int dist) throws InterruptedException {
        slidemotor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidemotor.setPower(power);
        slidemotor.runToPosition(slidemotor.getCurrentPosition() + dist, true);
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
