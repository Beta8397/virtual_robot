package com.acmerobotics.dashboard;

import androidx.annotation.Nullable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

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

    public void clearTelemetry() {}


    private final Telemetry.Item nullItem = new Telemetry.Item() {
        @Override
        public Telemetry.Item addData(String caption, Object value) {
            return this;
        }

        @Override
        public <T> Telemetry.Item addData(String caption, Func<T> valueProducer) {
            return this;
        }

        @Override
        public <T> Telemetry.Item addData(String caption, String format, Func<T> valueProducer) {
            return this;
        }

        @Override
        public String getCaption() {
            return "";
        }

        @Override
        public Telemetry.Item setCaption(String caption) {
            return this;
        }

        @Override
        public Telemetry.Item setValue(String format, Object... args) {
            return this;
        }

        @Override
        public Telemetry.Item setValue(Object value) {
            return this;
        }

        @Override
        public <T> Telemetry.Item setValue(Func<T> valueProducer) {
            return this;
        }

        @Override
        public <T> Telemetry.Item setValue(String format, Func<T> valueProducer) {
            return this;
        }

        @Override
        public Telemetry.Item setRetained(@Nullable Boolean retained) {
            return this;
        }

        @Override
        public boolean isRetained() {
            return false;
        }

        @Override
        public Telemetry.Item addData(String caption, String format, Object... args) {
            return this;
        }
    };

    private final Telemetry.Line nullLine = new Telemetry.Line() {
        @Override
        public Telemetry.Item addData(String caption, String format, Object... args) {
            return nullItem;
        }

        @Override
        public Telemetry.Item addData(String caption, Object value) {
            return nullItem;
        }

        @Override
        public <T> Telemetry.Item addData(String caption, Func<T> valueProducer) {
            return nullItem;
        }

        @Override
        public <T> Telemetry.Item addData(String caption, String format, Func<T> valueProducer) {
            return nullItem;
        }
    };

    public Telemetry getTelemetry(){
        return new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return nullItem;
            }

            @Override
            public Item addData(String caption, Object value) {
               return nullItem;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
               return nullItem;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
               return nullItem;
            }

            @Override
            public boolean removeItem(Item item) {
                return true;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
               return nullItem;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
               return nullLine;
            }

            @Override
            public Line addLine(String lineCaption) {
               return nullLine;
            }

            @Override
            public boolean removeLine(Line line) {
                return true;
            }

            @Override
            public boolean isAutoClear() {
                return true;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
               return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
               return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
               return new Log() {
                   @Override
                   public int getCapacity() {
                       return 0;
                   }

                   @Override
                   public void setCapacity(int capacity) {

                   }

                   @Override
                   public DisplayOrder getDisplayOrder() {
                       return DisplayOrder.OLDEST_FIRST;
                   }

                   @Override
                   public void setDisplayOrder(DisplayOrder displayOrder) {

                   }

                   @Override
                   public void add(String entry) {

                   }

                   @Override
                   public void add(String format, Object... args) {

                   }

                   @Override
                   public void clear() {

                   }
               };
            }
        };
    }

    public void setTelemetryTransmissionInterval(int telemetryTransmissionInterval){}
    public int getTelemetryTransmissionInterval(){return 0;}

    public void startCameraStream(CameraStreamSource cameraStreamSource, int i) {}

    public void stopCameraStream() {}


}