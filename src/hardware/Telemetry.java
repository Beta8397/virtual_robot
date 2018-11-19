package hardware;


import controller.VirtualRobotController;

public class Telemetry extends VirtualRobotController.TelemetryBase {
    private final StringBuilder data = new StringBuilder(200);

    public Telemetry(){
        update();
    }

    public void addData(String caption, String fmt, Object... data){
        this.data.append(caption + ":");
        String s = String.format(fmt, data);
        this.data.append(s + "\n");
    }

    public void addData(String caption, Object data){
        this.data.append(caption + ":" + data.toString() + "\n");
    }

    public void update(){
        setText(data.toString());
        data.setLength(0);
    }
}
