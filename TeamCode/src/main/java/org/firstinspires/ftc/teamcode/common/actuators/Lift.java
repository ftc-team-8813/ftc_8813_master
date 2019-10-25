package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;

public class Lift {
    public PIDMotor slidemotor;
    private Switch bottomswitch;
    private Switch topswitch;

    public Lift(PIDMotor slidemotor, Switch topswitch, Switch bottomswitch){
        this.slidemotor = slidemotor;
        this.topswitch = topswitch;
        this.bottomswitch = bottomswitch;
    }

    public void raiseLift(double power){
        if (power > 0 && !topswitch.pressed()){
            if (slidemotor.isHolding()){
                slidemotor.stopHolding();
            }
            slidemotor.getMotor().setPower(power);
        } else if (power < 0 && !bottomswitch.pressed()){
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

    public void getCurrentPos(){
        slidemotor.getCurrentPosition();
    }


}
