package org.firstinspires.ftc.teamcode.common.util;

import android.support.annotation.NonNull;
import android.util.JsonWriter;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

// TODO better error handling
// We might migrate the configuration to this in the future as it is writable by the program,
// unlike Config
public class DataStorage
{
    private JsonObject object;
    private File file;

    public DataStorage(@NonNull File f)
    {
        file = f;
        if (f.exists())
        {
            try
            {
                load();
            } catch (IOException e)
            {
                throw new IllegalArgumentException("Cannot read file " + f + ": " + e.getMessage(), e);
            }
        }
        else
        {
            object = new JsonObject();
        }
    }

    private void load() throws IOException
    {
        FileReader f = new FileReader(file);
        JsonParser parser = new JsonParser();
        try
        {
            object = (JsonObject)parser.parse(f);
        }
        catch (ClassCastException e)
        {
            throw new IOException("Malformed data file -- root is not a JSON object!");
        }
        catch (JsonParseException e)
        {
            throw new IOException("Malformed JSON data");
        }
        finally
        {
            f.close();
        }
    }

    private void store() throws IOException
    {
        FileWriter w = new FileWriter(file);
        Gson gson = new Gson();
        gson.toJson(object, w);
        w.close();
    }

    public void save()
    {
        try
        {
            store();
        }
        catch (IOException e)
        {
            throw new IllegalStateException("Cannot save data file to " + file);
        }
    }

    public void addJson(String name, JsonElement element)
    {
        object.add(name, element);
    }

    public JsonElement getJson(String name)
    {
        return object.get(name);
    }

    public void addNumber(String name, double value)
    {
        addJson(name, new JsonPrimitive(value));
    }

    public void addBoolean(String name, boolean value)
    {
        addJson(name, new JsonPrimitive(value));
    }

    public void addString(String name, String value)
    {
        addJson(name, new JsonPrimitive(value));
    }

    public void addArray(String name, int[] value)
    {
        JsonArray array = new JsonArray();
        for (int i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public void addArray(String name, long[] value)
    {
        JsonArray array = new JsonArray();
        for (long i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public void addArray(String name, double[] value)
    {
        JsonArray array = new JsonArray();
        for (double i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public <T extends Number> void addArray(String name, T[] value)
    {
        JsonArray array = new JsonArray();
        for (T i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public void addArray(String name, Boolean[] value)
    {
        JsonArray array = new JsonArray();
        for (Boolean i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public void addArray(String name, String[] value)
    {
        JsonArray array = new JsonArray();
        for (String i : value)
        {
            array.add(i);
        }
        addJson(name, array);
    }

    public Number getNumber(String name)
    {
        JsonElement obj = object.get(name);
        if (obj == null) return null;
        return obj.getAsNumber();
    }

    public int getInt(String name, int def)
    {
        JsonElement obj = object.get(name);
        if (obj == null) return def;
        return obj.getAsInt();
    }

    public double getDouble(String name, double def)
    {
        JsonElement obj = object.get(name);
        if (obj == null) return def;
        return obj.getAsDouble();
    }

    public boolean getBoolean(String name, boolean def)
    {
        JsonElement obj = object.get(name);
        if (obj == null) return def;
        return obj.getAsBoolean();
    }

    public String getString(String name)
    {
        JsonElement obj = object.get(name);
        if (obj == null) return null;
        return obj.getAsString();
    }

    public int[] getIntArray(String name)
    {
        JsonArray obj = (JsonArray)object.get(name);
        if (obj == null) return null;
        int[] out = new int[obj.size()];
        int i = 0;
        for (JsonElement e : obj)
        {
            out[i] = e.getAsInt();
            i++;
        }
        return out;
    }

    public long[] getLongArray(String name)
    {
        JsonArray obj = (JsonArray)object.get(name);
        if (obj == null) return null;
        long[] out = new long[obj.size()];
        int i = 0;
        for (JsonElement e : obj)
        {
            out[i] = e.getAsLong();
            i++;
        }
        return out;
    }

    public double[] getDoubleArray(String name)
    {
        JsonArray obj = (JsonArray)object.get(name);
        if (obj == null) return null;
        double[] out = new double[obj.size()];
        int i = 0;
        for (JsonElement e : obj)
        {
            out[i] = e.getAsDouble();
            i++;
        }
        return out;
    }

    public boolean[] getBooleanArray(String name)
    {
        JsonArray obj = (JsonArray)object.get(name);
        if (obj == null) return null;
        boolean[] out = new boolean[obj.size()];
        int i = 0;
        for (JsonElement e : obj)
        {
            out[i] = e.getAsBoolean();
            i++;
        }
        return out;
    }

    public String[] getStringArray(String name)
    {
        JsonArray obj = (JsonArray)object.get(name);
        if (obj == null) return null;
        String[] out = new String[obj.size()];
        int i = 0;
        for (JsonElement e : obj)
        {
            out[i] = e.getAsString();
            i++;
        }
        return out;
    }

}
