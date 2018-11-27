package virtual_robot.hardware;


import virtual_robot.controller.VirtualRobotController;

/**
 * Simulates (in a limited way) the FTC SDK Telemetry capability
 */
public class Telemetry extends VirtualRobotController.TelemetryBase {
    private final StringBuilder data = new StringBuilder(200);

    public Telemetry(){
        update();
    }

    /**
     * Add data to telemetry (note-must call update() to cause the data to be displayed)
     * @param caption The caption for this telemetry entry.
     * @param fmt Format string, for formatting the data.
     * @param data The data to be formatted by the format string.
     */
    public void addData(String caption, String fmt, Object... data){
        this.data.append(caption + ": ");
        String s = String.format(fmt, data);
        this.data.append(s + "\n");
    }

    /**
     * Add single data object to telemetry, with a caption (note-must call update() to cause the data to be displayed)
     * @param caption The caption for this telemetry entry.
     * @param data The data for this telemetry entry.
     */
    public void addData(String caption, Object data){
        this.data.append(caption + ":" + data.toString() + "\n");
    }


    /**
     * Replace any data currently displayed on telemetry with all data that has been added since the previous call to
     * update().
     */
    public void update(){
        setText(data.toString());
        data.setLength(0);
    }
}
