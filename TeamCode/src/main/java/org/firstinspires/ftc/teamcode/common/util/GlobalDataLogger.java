package org.firstinspires.ftc.teamcode.common.util;

import android.support.annotation.NonNull;
import android.util.Log;

import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;
import java.util.zip.GZIPOutputStream;

/**
 * Global Data Logger -- Writes gzipped text files that follow this format: <br>
 * <pre><code>
 *     [time]   data1   data2   data3   ... data(n)
 *     [time]   data1   data2   data3   ... data(n)
 *     ...
 *     name1    name2   name3   ... name(n)
 * </code></pre>
 * <p>
 * Fields are separated by tabs, times are in nanoseconds since the logger started, and the channel
 * headings are listed at the end of the file.
 * </p>
 * <p>
 * Channels may be added at any time, but they may not be removed for the lifetime of the logger. As
 * channels are added, they will appear as additional columns, like so: <br>
 * <pre><code>
 *     [time]   data1
 *     ...
 *     [time]   data1
 *        (new channel added)
 *     [time]   data1   data2
 *     ...
 *     name1    name2   ... name(n)
 * </code></pre>
 * </p>
 * <p>
 * Inside fields, some characters are escaped to avoid ambiguity:
 * <pre><code>
 *     \ (backslash)   -> \\
 *     (tab)           -> \t
 * </code></pre>
 * </p>
 */
public class GlobalDataLogger
{
    
    private static GlobalDataLogger instance;
    
    private static class DummyDataLogger extends GlobalDataLogger
    {
    
        private DummyDataLogger() throws IOException
        {
            super("/dev/null");
        }
    
        @Override
        public synchronized void addChannel(String name, Callable<String> callback)
        {
            log.w("Not adding channel: %s", name);
        }
    
        @Override
        public synchronized void start(int interval) { }
    
        @Override
        public synchronized void stop() { }
    }
    
    /**
     * Get the current GlobalDataLogger. Returns a dummy if the logger is closed or has not been
     * initialized yet.
     * @return The global logger instance.
     */
    @NonNull
    public static GlobalDataLogger instance()
    {
        if (instance == null)
        {
            Log.w("GlobalDataLogger", "Returning dummy logger");
            try
            {
                return new DummyDataLogger();
            } catch (IOException e)
            {
                Log.e("GlobalDataLogger", "Failed to create dummy logger");
                return null; // This should never happen
            }
        }
        return instance;
    }
    
    /**
     * Initialize the data logger. Closes any running
     * @param filename The path of the file to write to
     * @return The newly created logger
     * @throws IOException if an error occurs while opening the file
     */
    public static GlobalDataLogger initialize(String filename) throws IOException
    {
        if (instance != null) instance.stop();
        instance = new GlobalDataLogger(filename);
        init();
        return instance;
    }
    
    private static void init()
    {
        VMStats stats = new VMStats(1); // Add VMStats logs
        instance().addChannel("GlobalThreadPool Thread Count", () -> "" + GlobalThreadPool.instance().getTaskCount());
    }
    
    private final Writer writer;
    private volatile List<Channel> channels;
    private Future<?> logDaemon;
    protected Logger log;
    
    private class Channel
    {
        final String name;
        final Callable<String> callback;
        
        Channel(String name, Callable<String> callback)
        {
            this.name = name;
            this.callback = callback;
        }
    }
    
    
    private GlobalDataLogger(String filename) throws IOException
    {
        writer = new OutputStreamWriter(new BufferedOutputStream(new GZIPOutputStream(new FileOutputStream(filename))));
        channels = Collections.synchronizedList(new ArrayList<>());
        log = new Logger("GlobalDataLogger");
    }
    
    ///////
    // Special characters that need escaping
    // [tab] -> \t    -- Delimiter
    //   \   -> \\    -- Escaping
    
    /**
     * Add a channel.
     * @param name The name of the channel.
     * @param callback A callback function to request log data. This should run as fast as possible.
     */
    public synchronized void addChannel(String name, Callable<String> callback)
    {
        channels.add(new Channel(name, callback));
    }
    
    /**
     * Start the logging process.
     * @param interval The interval, in milliseconds, to delay between samples. It is recommended to
     *                 make this greater than zero.
     */
    public synchronized void start(int interval)
    {
        logDaemon = GlobalThreadPool.instance().start(() ->
        {
            long start = System.nanoTime();
            boolean interrupt = false;
            try
            {
                while (true)
                {
                    long loopstart = System.nanoTime();
                    List<String> data = new ArrayList<>();
                    for (Channel c : new Vector<>(channels))
                    {
                        try
                        {
                            data.add(c.callback.call());
                        }
                        catch (InterruptedException e)
                        {
                            interrupt = true;
                            data.add("~interrupt~");
                        }
                        catch (Exception e)
                        {
                            data.add("~error " + e.getClass().getSimpleName() + "~");
                        }
                    }
                    data.add(0, Double.toString(1000000000.0 / (System.nanoTime() - loopstart)));
                    writeLogLine(System.nanoTime() - start, data);
                    if (interrupt) break;
                    Thread.sleep(interval);
                }
            }
            catch (InterruptedException | IOException e)
            {
                log.i("Interrupted");
            }
            finally
            {
                close();
            }
        });
    }
    
    private String sanitize(String input)
    {
        if (input == null) return "null";
        else return input.replaceAll("\\\\", "\\\\").replaceAll("\t", "\\t");
    }
    
    private void writeLogLine(long time, List<String> data) throws IOException
    {
        if (time >= 0) data.add(0, Long.toString(time));
        
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < data.size(); i++)
        {
            out.append(sanitize(data.get(i)));
            if (i < data.size() -1) out.append("\t");
            else out.append("\n");
        }
        writer.write(out.toString());
    }
    
    public synchronized void stop()
    {
        if (logDaemon != null) logDaemon.cancel(true);
        else
        {
            try
            {
                writer.close();
            }
            catch (IOException e)
            {
                log.e("Failed to close log file");
                log.e(e);
            }
            finally
            {
                instance = null;
            }
        }
    }
    
    private void close()
    {
        try
        {
            List<String> names = new ArrayList<>();
            names.add("Frames per second");
            channels.forEach((channel) -> names.add(channel.name));
            
            writeLogLine(-1, names);
            writer.flush();
            writer.close();
        }
        catch (IOException e)
        {
            log.e("Failed to close data log file");
            log.e(e);
        }
        finally
        {
            instance = null;
        }
    }
}
