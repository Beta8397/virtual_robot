package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Wrapper for FtcDashboard.getTelemetry() to not throw UnsupportedOperationExceptions.
 */
public class FtcDashboardTelemetry implements Telemetry {
    private final Telemetry telemetry;
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

    public FtcDashboardTelemetry() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public Telemetry.Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(format, args));
    }

    @Override
    public Telemetry.Item addData(String caption, Object value) {
        telemetry.addData(caption, value);
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

    @Override
    public boolean removeItem(Telemetry.Item item) {
        return false;
    }

    @Override
    public void clear() {
        telemetry.clear();
    }

    @Override
    public void clearAll() {
        telemetry.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return nullItem;
    }

    @Override
    public boolean removeAction(Object token) {
        return true;
    }

    @Override
    public void speak(String text) {
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
    }

    @Override
    public boolean update() {
        return telemetry.update();
    }

    @Override
    public Telemetry.Line addLine() {
        return nullLine;
    }

    @Override
    public Telemetry.Line addLine(String lineCaption) {
        telemetry.addLine(lineCaption);
        return nullLine;
    }

    @Override
    public boolean removeLine(Telemetry.Line line) {
        return false;
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
        return telemetry.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetry.setMsTransmissionInterval(msTransmissionInterval);
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
        return " : ";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
    }

    @Override
    public Log log() {
        return telemetry.log();
    }
}
