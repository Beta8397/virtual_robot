package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FtcDashboard {
    private static FtcDashboard instance;

    private FtcDashboard(){}

    public static FtcDashboard getInstance() {
        if (instance == null) {
            instance = new FtcDashboard();
        }
        return instance;
    }

    public void sendTelemetryPacket(TelemetryPacket telemetryPacket) {}

    public Telemetry getTelemetry(){
        return null;
    }

    public void setTelemetryTransmissionInterval(int telemetryTransmissionInterval){}


}