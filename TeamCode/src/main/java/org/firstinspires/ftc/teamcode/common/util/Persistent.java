package org.firstinspires.ftc.teamcode.common.util;

import java.util.HashMap;
import java.util.Map;

/**
 * Keeps objects across the autonomous-TeleOp boundary so they don't have to be re-initialized.
 * Main use is the IMU, which causes significant lag on startup and should be initialized in
 * autonomous.
 */

public class Persistent
{
    private static Map<String, Object> objects = new HashMap<>();
    
    public static void clear()
    {
        objects.clear();
    }
    
    public static Object get(String name)
    {
        return objects.get(name);
    }
    
    public static void put(String name, Object o)
    {
        objects.put(name, o);
    }
}
