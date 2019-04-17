package org.firstinspires.ftc.teamcode.common.util;

import android.support.annotation.NonNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.NoSuchElementException;

public class Profiler
{
    private class Item implements Comparable<Item>
    {
        String name;
        List<Item> subItems = new ArrayList<>();
        Item parent;
        long nanos;
        long start;

        Item(String name)
        {
            this.name = name;
            start = System.nanoTime();
        }

        void addItem(Item sub)
        {
            Item p = this;
            if (sub.parent != null) throw new IllegalArgumentException("Only new items can be added");
            while (p != null)
            {
                if (sub == p) throw new IllegalArgumentException("Recursion is not allowed");
                p = p.parent;
            }
            subItems.add(sub);
            sub.parent = this;
        }

        void addTime(long nanos)
        {
            Item p = this;
            while (p != null)
            {
                p.nanos += nanos;
                p = p.parent;
            }
        }

        String getFullName()
        {
            if (parent == null) return name + "/";
            else return parent.getFullName() + name + "/";
        }

        @Override
        public int compareTo(@NonNull Item o)
        {
            return Long.compare(nanos, o.nanos);
        }
    }

    private Item root;
    private Item curr;
    private Logger log;
    private boolean disabled;

    public Profiler()
    {
        log = new Logger("Profiler");
        root = new Item("");
        curr = root;
    }

    public Profiler disable()
    {
        disabled = true;
        return this;
    }

    public void start(String name)
    {
        if (disabled) return;
        Item n = new Item(name);
        curr.addTime(System.nanoTime() - curr.start);
        curr.start = System.nanoTime();
        curr.addItem(n);
        curr = n;
        log.d("Starting %s", n.getFullName());
    }

    public void end()
    {
        if (disabled) return;
        if (curr.parent == null) throw new NoSuchElementException("Stack underflow");
        curr.addTime(System.nanoTime() - curr.start);
        curr = curr.parent;
        curr.start = System.nanoTime();
    }

    public void finish()
    {
        if (disabled) return;
        while (curr.parent != null) end();
        log.d("");
        log.d("Profiling summary");
        log.d("-----------------------------------");
        printItem(root);
    }

    private void printItem(Item i)
    {
        log.d("%s -- %.4f s", i.getFullName(), (double)i.nanos / 1E9);
        Collections.sort(i.subItems, Collections.reverseOrder());
        for (Item sub : i.subItems)
        {
            printItem(sub);
        }
    }
}
