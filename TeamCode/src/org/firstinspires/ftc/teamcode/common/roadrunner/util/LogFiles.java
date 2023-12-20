package org.firstinspires.ftc.teamcode.common.roadrunner.util;

//import android.annotation.SuppressLint;
import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;
//import com.qualcomm.robotcore.util.WebHandlerManager;

//import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Objects;

//import fi.iki.elonen.NanoHTTPD;

// Incompatible with virtual_robot, methods have been nullified
public final class LogFiles {
//    private static final File ROOT =
//            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/logs/");
//    private static final File ROOT = null;

//    public static LogFile log = new LogFile("uninitialized");
//    private static final OpModeManagerNotifier.Notifications notifHandler = new OpModeManagerNotifier.Notifications() {
//        @SuppressLint("SimpleDateFormat")
//        final DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS");
//
//        final ObjectWriter jsonWriter = new ObjectMapper(new JsonFactory())
//                .writerWithDefaultPrettyPrinter();
//
//        @Override
//        public void onOpModePreInit(OpMode opMode) {
//            log = new LogFile(opMode.getClass().getCanonicalName());
//
//            // clean up old files
//            File[] fs = Objects.requireNonNull(ROOT.listFiles());
//            Arrays.sort(fs, (a, b) -> Long.compare(a.lastModified(), b.lastModified()));
//            long totalSizeBytes = 0;
//            for (File f : fs) {
//                totalSizeBytes += f.length();
//            }
//
//            int i = 0;
//            while (i < fs.length && totalSizeBytes >= 32 * 1000 * 1000) {
//                totalSizeBytes -= fs[i].length();
//                if (!fs[i].delete()) {
//                    RobotLog.setGlobalErrorMsg("Unable to delete file " + fs[i].getAbsolutePath());
//                }
//                ++i;
//            }
//        }
//
//        @Override
//        public void onOpModePreStart(OpMode opMode) {
//            log.nsStart = System.nanoTime();
//        }
//
//        @Override
//        public void onOpModePostStop(OpMode opMode) {
//            log.nsStop = System.nanoTime();
//
//            if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
//                //noinspection ResultOfMethodCallIgnored
//                ROOT.mkdirs();
//
//                String filename = dateFormat.format(new Date(log.msInit)) + "__" + opMode.getClass().getSimpleName() + ".json";
//                File file = new File(ROOT, filename);
//                try {
//                    jsonWriter.writeValue(file, log);
//                } catch (IOException e) {
//                    RobotLog.setGlobalErrorMsg(new RuntimeException(e),
//                            "Unable to write data to " + file.getAbsolutePath());
//                }
//            }
//        }
//    };

    public static void record(
            Pose2d targetPose, Pose2d pose, double voltage,
            List<Integer> lastDriveEncPositions, List<Integer> lastDriveEncVels, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels
    ) {
//        long nsTime = System.nanoTime();
//        if (nsTime - log.nsStart > 3 * 60 * 1_000_000_000L) {
//            return;
//        }
//
//        log.nsTimes.add(nsTime);
//
//        log.targetXs.add(targetPose.getX());
//        log.targetYs.add(targetPose.getY());
//        log.targetHeadings.add(targetPose.getHeading());
//
//        log.xs.add(pose.getX());
//        log.ys.add(pose.getY());
//        log.headings.add(pose.getHeading());
//
//        log.voltages.add(voltage);
//
//        while (log.driveEncPositions.size() < lastDriveEncPositions.size()) {
//            log.driveEncPositions.add(new ArrayList<>());
//        }
//        while (log.driveEncVels.size() < lastDriveEncVels.size()) {
//            log.driveEncVels.add(new ArrayList<>());
//        }
//        while (log.trackingEncPositions.size() < lastTrackingEncPositions.size()) {
//            log.trackingEncPositions.add(new ArrayList<>());
//        }
//        while (log.trackingEncVels.size() < lastTrackingEncVels.size()) {
//            log.trackingEncVels.add(new ArrayList<>());
//        }
//
//        for (int i = 0; i < lastDriveEncPositions.size(); i++) {
//            log.driveEncPositions.get(i).add(lastDriveEncPositions.get(i));
//        }
//        for (int i = 0; i < lastDriveEncVels.size(); i++) {
//            log.driveEncVels.get(i).add(lastDriveEncVels.get(i));
//        }
//        for (int i = 0; i < lastTrackingEncPositions.size(); i++) {
//            log.trackingEncPositions.get(i).add(lastTrackingEncPositions.get(i));
//        }
//        for (int i = 0; i < lastTrackingEncVels.size(); i++) {
//            log.trackingEncVels.get(i).add(lastTrackingEncVels.get(i));
//        }
    }

//    @WebHandlerRegistrar
//    public static void registerRoutes(Context context, WebHandlerManager manager) {
//        //noinspection ResultOfMethodCallIgnored
//        ROOT.mkdirs();
//
//        // op mode manager only stores a weak reference, so we need to keep notifHandler alive ourselves
//        // don't use @OnCreateEventLoop because it's unreliable
//        OpModeManagerImpl.getOpModeManagerOfActivity(
//                AppUtil.getInstance().getActivity()
//        ).registerListener(notifHandler);
//
//        manager.register("/logs", session -> {
//            StringBuilder sb = new StringBuilder();
//            sb.append("<!doctype html><html><head><title>Logs</title></head><body><ul>");
//            File[] fs = Objects.requireNonNull(ROOT.listFiles());
//            Arrays.sort(fs, (a, b) -> Long.compare(b.lastModified(), a.lastModified()));
//            for (File f : fs) {
//                sb.append("<li><a href=\"/logs/download?file=");
//                sb.append(f.getName());
//                sb.append("\" download=\"");
//                sb.append(f.getName());
//                sb.append("\">");
//                sb.append(f.getName());
//                sb.append("</a></li>");
//            }
//            sb.append("</ul></body></html>");
//            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
//                    NanoHTTPD.MIME_HTML, sb.toString());
//        });
//
//        manager.register("/logs/download", session -> {
//            String[] pairs = session.getQueryParameterString().split("&");
//            if (pairs.length != 1) {
//                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
//                        NanoHTTPD.MIME_PLAINTEXT, "expected one query parameter, got " + pairs.length);
//            }
//
//            String[] parts = pairs[0].split("=");
//            if (!parts[0].equals("file")) {
//                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
//                        NanoHTTPD.MIME_PLAINTEXT, "expected file query parameter, got " + parts[0]);
//            }
//
//            File f = new File(ROOT, parts[1]);
//            if (!f.exists()) {
//                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
//                        NanoHTTPD.MIME_PLAINTEXT, "file " + f + " doesn't exist");
//            }
//
//            return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
//                    "application/json", new FileInputStream(f));
//        });
//    }

    public static class LogFile {
        public String version = "quickstart1 v2";

        public String opModeName;
        public long msInit = System.currentTimeMillis();
        public long nsInit = System.nanoTime();
        public long nsStart, nsStop;

        public List<Long> nsTimes = new ArrayList<>();

        public List<Double> targetXs = new ArrayList<>();
        public List<Double> targetYs = new ArrayList<>();
        public List<Double> targetHeadings = new ArrayList<>();

        public List<Double> xs = new ArrayList<>();
        public List<Double> ys = new ArrayList<>();
        public List<Double> headings = new ArrayList<>();

        public List<Double> voltages = new ArrayList<>();

        public List<List<Integer>> driveEncPositions = new ArrayList<>();
        public List<List<Integer>> driveEncVels = new ArrayList<>();
        public List<List<Integer>> trackingEncPositions = new ArrayList<>();
        public List<List<Integer>> trackingEncVels = new ArrayList<>();

        public LogFile(String opModeName) {
            this.opModeName = opModeName;
        }
    }
}
