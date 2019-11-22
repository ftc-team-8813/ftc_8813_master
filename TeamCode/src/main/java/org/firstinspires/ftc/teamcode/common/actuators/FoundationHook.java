package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

public class FoundationHook {
    public Servo hook;

    public FoundationHook(Servo hook){
        this.hook = hook;
    }

    public void moveHookDown(){
        hook.setPosition(0.8);
    }

    public void moveHookUp(){
        hook.setPosition(0);
    }
}
