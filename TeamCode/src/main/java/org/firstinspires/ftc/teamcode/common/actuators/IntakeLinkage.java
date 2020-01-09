package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.DataStorage;

public class IntakeLinkage
{
    private Servo linkage_l, linkage_r;
    private double linkage_l_in, linkage_l_med, linkage_l_out;
    private double linkage_r_in, linkage_r_med, linkage_r_out;
    
    public static final int IN = 0;
    public static final int MED = 1;
    public static final int OUT = 2;

    public IntakeLinkage(Servo linkage_l, Servo linkage_r, DataStorage positions)
    {
        this.linkage_l = linkage_l;
        this.linkage_r = linkage_r;
        
        //#position 'intake l' in
        linkage_l_in = positions.getDouble("intake l.in", 1);
        //#position 'intake l' med
        linkage_l_med = positions.getDouble("intake l.med", 0.5);
        //#position 'intake l' out
        linkage_l_out = positions.getDouble("intake l.out", 0);
    
        //#position 'intake r' in
        linkage_r_in = positions.getDouble("intake r.in", 1);
        //#position 'intake r' med
        linkage_r_med = positions.getDouble("intake r.med", 0.5);
        //#position 'intake r' out
        linkage_r_out = positions.getDouble("intake r.out", 0);
    }

    public void moveLinkageIn()
    {
        moveLinkage(IN, IN);
    }

    public void moveLinkageOut()
    {
        moveLinkage(OUT, OUT);
    }
    
    public void moveLinkage(int left, int right)
    {
        if (left == IN)       linkage_l.setPosition(linkage_l_in);
        else if (left == MED) linkage_l.setPosition(linkage_l_med);
        else                  linkage_l.setPosition(linkage_l_out);
    
        if (right == IN)      linkage_r.setPosition(linkage_r_in);
        else if (right == MED) linkage_r.setPosition(linkage_r_med);
        else                  linkage_r.setPosition(linkage_r_out);
    }
}
