package org.firstinspires.ftc.teamcode.autonomous.util;

import android.os.Environment;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Holder for configurable variables.
 */

public class Config {
    public static final String storageDir =
            Environment.getExternalStorageDirectory().getAbsolutePath() + "/Team8813/";
    private Properties properties;

    public Config(String filename) throws IOException {
        new File(storageDir).mkdirs();
        this.properties = new Properties();
        properties.load(new FileInputStream(storageDir + filename));
    }


    public int getInt(String name, int def) {
        try {
            return Integer.parseInt(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e) {
            return def;
        }
    }

    public double getDouble(String name, double def) {
        try {
            return Double.parseDouble(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e) {
            return def;
        }
    }

    public String getString(String name, String def) {
        if (properties.getProperty(name) != null)
            return properties.getProperty(name);
        else
            return def;
    }

    public boolean getBoolean(String name, boolean def) {
        if (properties.getProperty(name) != null)
            return Boolean.parseBoolean(properties.getProperty(name));
        else
            return def;
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
            } else return null;
        } catch (NumberFormatException e) {
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
            } else return null;
        } catch (NumberFormatException e) {
            return null;
        }
    }

    public String[] getStringArray(String name) {
        if (properties.getProperty(name) != null)
            return csv(properties.getProperty(name));
        else return null;
    }

    public boolean[] getBooleanArray(String name) {
        if (properties.getProperty(name) != null) {
            String[] values = csv(properties.getProperty(name));
            boolean[] out = new boolean[values.length];
            for (int i = 0; i < values.length; i++) {
                out[i] = Boolean.parseBoolean(values[i]);
            }
            return out;
        } else return null;
    }

    private String[] csv(String s) {
        String[] values = s.split(",");
        for (int i = 0; i < values.length; i++) {
            values[i] = values[i].trim();
        }
        return values;
    }


}
