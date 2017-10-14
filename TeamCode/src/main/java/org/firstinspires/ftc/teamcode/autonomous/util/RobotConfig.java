package org.firstinspires.ftc.teamcode.autonomous.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Not to be confused with Config. This reads the robot controller's device configuration
 * TODO: Need to finish
 */

public class RobotConfig {
    private static Map<String, Class<? extends Device>> knownDevices = new HashMap<>();
    static {
        knownDevices.put("LynxUsbDevice", ExpHubPortalDevice.class);
        knownDevices.put("LynxModule", ExpHubDevice.class);
    }

    public static abstract class Device {
        private String name;
        Device(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public static class ExpHubPortalDevice extends Device {

        private String serNum;
        private ExpHubDevice[] children;

        ExpHubPortalDevice(String name, String serNum, ExpHubDevice[] children) {
            super(name);
            this.serNum = serNum;
            this.children = children.clone();
        }

        public String getSerialNumber() {
            return serNum;
        }

        public ExpHubDevice[] getChildren() {
            return children.clone();
        }
    }

    public static class ExpHubDevice extends Device {

        private int port;

        ExpHubDevice(String name) {
            super(name);
        }
    }

    public RobotConfig() {

    }
}
