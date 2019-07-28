package org.firstinspires.ftc.robotcore.external;


import virtual_robot.controller.VirtualRobotController;

/**
 * Simulates (in a limited way) the FTC SDK Telemetry capability
 */
public interface Telemetry {

    StringBuilder data = new StringBuilder(200);

    /**
     * Add data for display by telemetry
     * @param caption
     * @param fmt   Standard Java format string for display of data
     * @param data  list of data items for display
     */
    public void addData(String caption, String fmt, Object... data);

    /**
     * Add data for display by telemetry
     * @param caption
     * @param data  data object to display
     */
    public void addData(String caption, Object data);

    /**
     * Clear the telemetry display, then write any data that has been added since the previous update.
     */
    public void update();
}
