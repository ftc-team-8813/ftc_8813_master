package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Config - Config file reader. Any configurable things (such as 'tweak' variables for quick adjustment)
 * should be put in the config file.
 */

public class Config {
    /** The base 'external' storage path (usually /sdcard or /storage/emulated/legacy etc.).
        This directory can be viewed when the phone is set in MTP mode                       */
    public static final String baseDir = Environment.getExternalStorageDirectory().getAbsolutePath();
    /** The 'storage' directory (the 'Team8813' folder in the file browser) */
    public static final String storageDir = baseDir + "/Team8813/";
    /** The name of the default config file (other ones can be used, but this is the default) */
    public static final String configFile = "config.properties";

    public static interface Encodeable {
        public boolean valid(String input);
        public void parse(String input);
    }

    private Properties properties;

    /**
     * Construct a configuration from a config file.
     * @param filename The name of the file to load the configuration from.
     */
    public Config(String filename) {
        new File(storageDir).mkdirs();
        try {
            this.properties = new Properties();
            properties.load(new FileInputStream(storageDir + filename));
        } catch (IOException e) {
            RobotLog.ee("Config", "No config file found");
        } catch (IllegalArgumentException e) {
            RobotLog.ee("Config", "Bad properties file");
        }
    }

    public int getInt(String name, int def) {
        try {
            return Integer.parseInt(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e) {
            RobotLog.ww("Config", "No int property named " + name + " found");
            return def;
        }
    }

    public double getDouble(String name, double def) {
        try {
            return Double.parseDouble(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e) {
            RobotLog.ww("Config", "No double property named " + name + " found");
            return def;
        }
    }

    public String getString(String name, String def) {
        if (properties.getProperty(name) != null) {
            return properties.getProperty(name);
        } else {
            RobotLog.ww("Config", "No string property named " + name + " found");
            return def;
        }
    }

    public boolean getBoolean(String name, boolean def) {
        if (properties.getProperty(name) != null) {
            return Boolean.parseBoolean(properties.getProperty(name));
        } else {
            RobotLog.ww("Config", "No boolean property named " + name + " found");
            return def;
        }
    }

    public int[] getIntArray(String name) {
        try {
            if (properties.getProperty(name) != null) {
                String[] values = csv(properties.getProperty(name));
                int[] out = new int[values.length];
                for (int i = 0; i < values.length; i++) {
                    out[i] = Integer.parseInt(values[i]);
                }
                return out;
            } else {
                RobotLog.ww("Config", "No int array property named " + name + " found");
                return null;
            }
        } catch (NumberFormatException e) {
            RobotLog.ww("Config", "No int array property named " + name + " found");
            return null;
        }
    }

    public double[] getDoubleArray(String name) {
        try {
            if (properties.getProperty(name) != null) {
                String[] values = csv(properties.getProperty(name));
                double[] out = new double[values.length];
                for (int i = 0; i < values.length; i++) {
                    out[i] = Double.parseDouble(values[i]);
                }
                return out;
            } else {
                RobotLog.ww("Config", "No double array property named " + name + " found");
                return null;
            }
        } catch (NumberFormatException e) {
            RobotLog.ww("Config", "No double array property named " + name + " found");
            return null;
        }
    }

    public String[] getStringArray(String name) {
        if (properties.getProperty(name) != null) {
            return csv(properties.getProperty(name));
        } else {
            RobotLog.ww("Config", "No string array property named " + name + " found");
            return null;
        }
    }

    public boolean[] getBooleanArray(String name) {
        if (properties.getProperty(name) != null) {
            String[] values = csv(properties.getProperty(name));
            boolean[] out = new boolean[values.length];
            for (int i = 0; i < values.length; i++) {
                out[i] = Boolean.parseBoolean(values[i]);
            }
            return out;
        } else {
            RobotLog.ww("Config", "No boolean array property named " + name + " found");
            return null;
        }
    }

    public Encodeable getEncodeable(String name, Class<? extends Encodeable> klass) {
        if (properties.getProperty(name) != null) {
            String value = properties.getProperty(name);
            Encodeable output;
            try {
                output = klass.newInstance();
            } catch (InstantiationException | IllegalAccessException e) {
                RobotLog.ww("Config", "Invalid encodeable " + klass.getName());
                return null;
            }
            if (output.valid(value)) {
                output.parse(value);
                return output;
            } else {
                RobotLog.ww("Config", "No valid " + klass.getSimpleName() + " property named " + name + " found");
            }
        }
        RobotLog.ww("Config", "No " + klass.getSimpleName() + " property named " + name + " found");
        return null;
    }

    private String[] csv(String s) {
        String[] values = s.split(",");
        for (int i = 0; i < values.length; i++) {
            values[i] = values[i].trim();
        }
        return values;
    }
}
