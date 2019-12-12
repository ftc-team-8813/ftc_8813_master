package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.Servo;

public class Blinkin
{
    public static final int RAINBOW               =  1;
    public static final int RAINBOW_PARTY         =  2;
    public static final int RAINBOW_OCEAN         =  3;
    public static final int RAINBOW_LAVA          =  4;
    public static final int RAINBOW_FOREST        =  5;
    public static final int RAINBOW_GLITTER       =  6;
    public static final int CONFETTI              =  7;
    public static final int SHOT_RED              =  8;
    public static final int SHOT_BLUE             =  9;
    public static final int SHOT_WHITE            =  10;
    public static final int SINELON_RAINBOW       =  11;
    public static final int SINELON_PARTY         =  12;
    public static final int SINELON_OCEAN         =  13;
    public static final int SINELON_LAVA          =  14;
    public static final int SINELON_FOREST        =  15;
    public static final int BPM_RAINBOW           =  16;
    public static final int BPM_PARTY             =  17;
    public static final int BPM_OCEAN             =  18;
    public static final int BPM_LAVA              =  19;
    public static final int BPM_FOREST            =  20;
    public static final int FIRE_MED              =  21;
    public static final int FIRE_LARGE            =  22;
    public static final int TWINKLE_RAINBOW       =  23;
    public static final int TWINKLE_PARTY         =  24;
    public static final int TWINKLE_OCEAN         =  25;
    public static final int TWINKLE_LAVA          =  26;
    public static final int TWINKLE_FOREST        =  27;
    public static final int WAVES_RAINBOW         =  28;
    public static final int WAVES_PARTY           =  29;
    public static final int WAVES_OCEAN           =  30;
    public static final int WAVES_LAVA            =  31;
    public static final int WAVES_FOREST          =  32;
    public static final int LARSON_RED            =  33;
    public static final int LARSON_GRAY           =  34;
    public static final int CHASE_RED             =  35;
    public static final int CHASE_BLUE            =  36;
    public static final int CHASE_GRAY            =  37;
    public static final int HEARTBEAT_RED         =  38;
    public static final int HEARTBEAT_BLUE        =  39;
    public static final int HEARTBEAT_WHITE       =  40;
    public static final int HEARTBEAT_GRAY        =  41;
    public static final int BREATH_RED            =  42;
    public static final int BREATH_BLUE           =  43;
    public static final int BREATH_GRAY           =  44;
    public static final int STROBE_RED            =  45;
    public static final int STROBE_BLUE           =  46;
    public static final int STROBE_GOLD           =  47;
    public static final int STROBE_WHITE          =  48;
    public static final int END2END2BLACK_COLOR1  =  49;
    public static final int LARSON_COLOR1         =  50;
    public static final int LIGHT_CHASE_COLOR1    =  51;
    public static final int HEARTBEAT_SLOW_COLOR1 =  52;
    public static final int HEARTBEAT_MED_COLOR1  =  53;
    public static final int HEARTBEAT_FAST_COLOR1 =  54;
    public static final int BREATH_SLOW_COLOR1    =  55;
    public static final int BREATH_FAST_COLOR1    =  56;
    public static final int SHOT_COLOR1           =  57;
    public static final int STROBE_COLOR1         =  58;
    public static final int END2END2BLACK_COLOR2  =  59;
    public static final int LARSON_COLOR2         =  60;
    public static final int LIGHT_CHASE_COLOR2    =  61;
    public static final int HEARTBEAT_SLOW_COLOR2 =  62;
    public static final int HEARTBEAT_MED_COLOR2  =  63;
    public static final int HEARTBEAT_FAST_COLOR2 =  64;
    public static final int BREATH_SLOW_COLOR2    =  65;
    public static final int BREATH_FAST_COLOR2    =  66;
    public static final int SHOT_COLOR2           =  67;
    public static final int STROBE_COLOR2         =  68;
    public static final int SPARKLE_1_ON_2        =  69;
    public static final int SPARKLE_2_ON_1        =  70;
    public static final int GRADIENT_1_2          =  71;
    public static final int BPM_1_2               =  72;
    public static final int END2END_1_TO_2        =  73;
    public static final int END2END               =  74;
    public static final int COLOR_1_2_NO_BLEND    =  75;
    public static final int TWINKLE_1_2           =  76;
    public static final int WAVES_1_2             =  77;
    public static final int SINELON_1_2           =  78;
    public static final int HOT_PINK              =  79;
    public static final int DARK_RED              =  80;
    public static final int RED                   =  81;
    public static final int RED_ORANGE            =  82;
    public static final int ORANGE                =  83;
    public static final int GOLD                  =  84;
    public static final int YELLOW                =  85;
    public static final int LAWN_GREEN            =  86;
    public static final int LIME                  =  87;
    public static final int DARK_GREEN            =  88;
    public static final int GREEN                 =  89;
    public static final int BLUE_GREEN            =  90;
    public static final int AQUA                  =  91;
    public static final int SKY_BLUE              =  92;
    public static final int DARK_BLUE             =  93;
    public static final int BLUE                  =  94;
    public static final int BLUE_VIOLET           =  95;
    public static final int VIOLET                =  96;
    public static final int WHITE                 =  97;
    public static final int GRAY                  =  98;
    public static final int DARK_GRAY             =  99;
    public static final int BLACK                 =  100;
    
    private Servo servo;
    
    public void setColor(int color)
    {
        servo.setPosition(color / 100.0);
    }
    
    public Blinkin(Servo servo)
    {
        this.servo = servo;
    }
}
