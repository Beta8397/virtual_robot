package virtual_robot.hardware;


import virtual_robot.controller.VirtualRobotController;

/**
 * Simulates (in a limited way) the FTC SDK Telemetry capability
 */
public interface Telemetry {

    StringBuilder data = new StringBuilder(200);

    public void addData(String caption, String fmt, Object... data);

    public void addData(String caption, Object data);

    public void update();
}
