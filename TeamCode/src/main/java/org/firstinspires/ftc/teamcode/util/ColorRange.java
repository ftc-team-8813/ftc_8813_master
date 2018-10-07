package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

public class ColorRange
{
    public static final int BLACK  = 0;
    public static final int WHITE  = 1;
    public static final int RED    = 2;
    public static final int ORANGE = 3;
    public static final int YELLOW = 4;
    public static final int GREEN  = 5;
    public static final int BLUE   = 6;
    public static final int PURPLE = 7;
    
    public static int getColor(int red, int green, int blue)
    {
        float[] hsv = new float[3];
        Color.colorToHSV(Color.rgb(red, green, blue), hsv);
        
        return 0;
    }
}
