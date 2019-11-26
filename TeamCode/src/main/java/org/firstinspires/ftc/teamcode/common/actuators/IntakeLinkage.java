package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLinkage {
    public Servo linkage;

    public IntakeLinkage(Servo linkage){
        this.linkage = linkage;
    }

    public void moveLinkageIn(){
        linkage.setPosition(1);
    }

    public void moveLinkageOut(){
        linkage.setPosition(0);
    }
}
