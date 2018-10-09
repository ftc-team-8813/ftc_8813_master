package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

public class ColorRange
{
    public static final int BLACK  = 0;
    public static final int GRAY   = 1;
    public static final int WHITE  = 2;
    public static final int RED    = 3;
    public static final int ORANGE = 4;
    public static final int YELLOW = 5;
    public static final int GREEN  = 6;
    public static final int BLUE   = 7;
    public static final int PURPLE = 8;
    private static final String[] colors = {
            "Black", "Gray", "White", "Red", "Orange", "Yellow", "Green", "Blue", "Purple"
    };
    
    public static int getColor(int red, int green, int blue)
    {
        float[] hsv = new float[3];
        Color.colorToHSV(Color.rgb(red, green, blue), hsv);
        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];
        if (s < .4 || v < .3)
        {
            if      (v > .8) return WHITE;
            else if (v > .3) return GRAY;
            else             return BLACK;
        }
        else
        {
            if      (h < 20)  return RED;
            else if (h < 25)  return ORANGE;
            else if (h < 75)  return YELLOW;
            else if (h < 165) return GREEN;
            else if (h < 270) return BLUE;
            else if (h < 315) return PURPLE;
            else              return RED;
        }
    }
    
    public static String toString(int col)
    {
        if (col < 0 || col > colors.length) return "Unknown";
        return colors[col];
    }
}
