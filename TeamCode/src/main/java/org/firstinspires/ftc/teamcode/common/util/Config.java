package org.firstinspires.ftc.teamcode.common.util;

import android.os.Environment;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Config - Config file reader. Any configurable things (such as 'tweak' variables for quick adjustment)
 * should be put in the config file.
 */

public class Config
{
    /**
     * The base 'external' storage path (usually /sdcard or /storage/emulated/legacy etc.).
     * This directory can be viewed when the phone is set in MTP mode
     */
    public static final String baseDir = Environment.getExternalStorageDirectory().getAbsolutePath();
    /**
     * The 'storage' directory (the 'Team8813' folder in the file browser)
     */
    public static final String storageDir = baseDir + "/Team8813/";
    /**
     * The name of the default config file (other ones can be used, but this is the default)
     */
    public static final String configFile = "config.properties";
    
    private static Logger log;
    
    /**
     * Encodeable - An interface for reading custom data types from a Config object. Examples:
     * <ul>
     * <li>Date parsing</li>
     * <li>JSON objects</li>
     * <li>Base64 parsing</li>
     * </ul>
     */
    public static interface Encodeable
    {
        /**
         * Parse the input string.
         *
         * @param input The input string
         * @throws IllegalArgumentException If the input was not valid
         */
        public void parse(String input);
    }
    
    private Properties properties;
    private boolean readSuccess = false;
    
    /**
     * Construct a configuration from a config file. If the file is invalid, unreadable, or corrupted,
     * a RobotLog message will be logged and the resulting Config will have no entries. Uses the
     * default .properties format
     *
     * @param filename The name of the file to load the configuration from.
     */
    public Config(String filename)
    {
        log = new Logger("Config");
        new File(storageDir).mkdirs();
        try (FileInputStream in = new FileInputStream(storageDir + filename))
        {
            this.properties = new Properties();
            properties.load(in);
            readSuccess = true; //Only if we get here did it succeed
        } catch (IOException e)
        {
            log.e("No config file '%s' found", filename);
        } catch (IllegalArgumentException e)
        {
            log.e("Bad properties file '%s'", filename);
        }
        log.v("Successfully loaded config file %s", filename);
    }
    
    public boolean readFailed()
    {
        return !readSuccess;
    }
    
    /**
     * Get a config entry as an int
     *
     * @param name The name of the entry
     * @param def  The default value if there is no entry with that name or the value is not a valid int
     * @return An integer value
     */
    public int getInt(String name, int def)
    {
        try
        {
            return Integer.parseInt(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e)
        {
            log.w("No valid integer property named %s", name);
            return def;
        }
    }
    
    /**
     * Get a config entry as a double
     *
     * @param name The name of the entry
     * @param def  The default value if there is no entry with that name or the value is not a valid double
     * @return A double value
     */
    public double getDouble(String name, double def)
    {
        try
        {
            return Double.parseDouble(properties.getProperty(name));
        } catch (NumberFormatException | NullPointerException e)
        {
            log.w("No valid double property named %s", name);
            return def;
        }
    }
    
    /**
     * Get a config entry as a String
     *
     * @param name The name of the entry
     * @param def  The default value if there is no entry with that name
     * @return A String
     */
    public String getString(String name, String def)
    {
        if (properties.getProperty(name) != null)
        {
            return properties.getProperty(name);
        } else
        {
            log.w("No string property named %s", name);
            return def;
        }
    }
    
    /**
     * Get a config value as a boolean. The value is parsed like {@link Boolean#parseBoolean(String)},
     * e.g. {@code false} unless the value is "true", ignoring case
     *
     * @param name The name of the entry
     * @param def  The default value if there is no entry with that name
     * @return A boolean
     */
    public boolean getBoolean(String name, boolean def)
    {
        if (properties.getProperty(name) != null)
        {
            return Boolean.parseBoolean(properties.getProperty(name));
        } else
        {
            log.w("No boolean property named %s", name);
            return def;
        }
    }
    
    /**
     * Get a config entry as a comma-separated array of ints.
     *
     * @param name The name of the entry
     * @return An int array, or null if no entry was found or it is invalid
     */
    public int[] getIntArray(String name)
    {
        try
        {
            if (properties.getProperty(name) != null)
            {
                String[] values = csv(properties.getProperty(name));
                int[] out = new int[values.length];
                for (int i = 0; i < values.length; i++)
                {
                    out[i] = Integer.parseInt(values[i]);
                }
                return out;
            } else
            {
                log.w("No int array property named %s", name);
                return null;
            }
        } catch (NumberFormatException e)
        {
            log.w("No valid int array property named %s", name);
            return null;
        }
    }
    
    /**
     * Get a config entry as a comma-separated array of doubles.
     *
     * @param name The name of the entry
     * @return A double array, or null if no entry was found or it is invalid
     */
    public double[] getDoubleArray(String name)
    {
        try
        {
            if (properties.getProperty(name) != null)
            {
                String[] values = csv(properties.getProperty(name));
                double[] out = new double[values.length];
                for (int i = 0; i < values.length; i++)
                {
                    out[i] = Double.parseDouble(values[i]);
                }
                return out;
            } else
            {
                log.w("No double array property named %s", name);
                return null;
            }
        } catch (NumberFormatException e)
        {
            log.w("No valid double array property named %s", name);
            return null;
        }
    }
    
    /**
     * Get a config entry as a comma-separated array of Strings. Commas inside the string are
     * interpreted as delimiters <em>even if they are escaped or quoted</em>.
     *
     * @param name The name of the entry
     * @return A String array, or null if no entry was found
     */
    public String[] getStringArray(String name)
    {
        if (properties.getProperty(name) != null)
        {
            return csv(properties.getProperty(name));
        } else
        {
            log.w("No string array property named %s", name);
            return null;
        }
    }
    
    /**
     * Get a config entry as a comma-separated array of booleans. Values are parsed like
     * {@link Boolean#parseBoolean(String)}, e.g. they are interpreted as 'false' unless the value
     * is "true", ignoring case.
     *
     * @param name The name of the entry
     * @return An int array, or null if no entry was found
     */
    public boolean[] getBooleanArray(String name)
    {
        if (properties.getProperty(name) != null)
        {
            String[] values = csv(properties.getProperty(name));
            boolean[] out = new boolean[values.length];
            for (int i = 0; i < values.length; i++)
            {
                out[i] = Boolean.parseBoolean(values[i]);
            }
            return out;
        } else
        {
            log.w("No boolean array property named %s", name);
            return null;
        }
    }
    
    /**
     * Read a config entry as a String and parse it using the specified {@link Encodeable}
     *
     * @param name  The name of the entry
     * @param klass The class of the {@link Encodeable} to use
     * @return An instance of the {@link Encodeable}, or null if:
     * <ul>
     * <li>The encodeable could not be instantiated (i.e. it has a private constructor)</li>
     * <li>The entry with the specified name could not be found</li>
     * <li>The entry with the specified name could not be parsed</li>
     * </ul>
     */
    public Encodeable getEncodeable(String name, Class<? extends Encodeable> klass)
    {
        if (properties.getProperty(name) != null)
        {
            String value = properties.getProperty(name);
            Encodeable output;
            try
            {
                output = klass.newInstance();
            } catch (InstantiationException | IllegalAccessException e)
            {
                log.w("Unable to instantiate class %s", klass.getName());
                return null;
            }
            try
            {
                output.parse(value);
                return output;
            } catch (IllegalArgumentException e)
            {
                log.w("No valid %s property named %s", klass.getName(), name);
            }
        }
        log.w("No %s property named %s", klass.getName(), name);
        return null;
    }
    
    private String[] csv(String s)
    {
        String[] values = s.split(",");
        for (int i = 0; i < values.length; i++)
        {
            values[i] = values[i].trim(); // allow for spaces (e.g. 1, 2   ,     3 instead of 1,2,3)
        }
        return values;
    }
}
