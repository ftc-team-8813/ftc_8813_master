package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;

public class Lift {
    public PIDMotor slidemotor;
    private Switch bottomswitch;

    public Lift(PIDMotor slidemotor, Switch bottomswitch){
        this.slidemotor = slidemotor;
        this.bottomswitch = bottomswitch;
    }

    public void raiseLift(double power){
        if (power > 0 && !bottomswitch.pressed()) {
            if (slidemotor.isHolding()) {
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power);
        } else if (power < 0){
            if (slidemotor.isHolding()){
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power * 0.5);
        } else {
            if (!slidemotor.isHolding()) {
                slidemotor.setPower(0.5);
                slidemotor.hold(slidemotor.getCurrentPosition());
            }
        }
    }

    public void getCurrentPos(){
        slidemotor.getCurrentPosition();
    }


}
