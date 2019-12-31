package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.DataStorage;

public class IntakeLinkage {
    public Servo linkage;
    private double linkage_in, linkage_out;

    public IntakeLinkage(Servo linkage, DataStorage positions){
        this.linkage = linkage;
        //#position 'intake linkage' in
        //#position 'intake linkage' out
        linkage_in = positions.getDouble("intake linkage.in", 1);
        linkage_out = positions.getDouble("intake linkage.out", 0);
    }

    public void moveLinkageIn(){
        linkage.setPosition(1);
    }

    public void moveLinkageOut(){
        linkage.setPosition(0);
    }
}
