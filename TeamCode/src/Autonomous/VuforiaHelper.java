//package Autonomous;
//
//import android.graphics.Bitmap;
//import android.graphics.Matrix;
//import android.os.Environment;
//import Misc.Log;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.vuforia.Image;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.camera.names.UnknownCameraNameImpl;
//
//import java.io.File;
//import java.io.FileOutputStream;
//import java.io.IOException;
//
//import static Autonomous.VisionHelperUltimateGoal.PHONE_CAMERA;
//import static Autonomous.VisionHelperUltimateGoal.WEBCAM;
//
///**
// * Created by robotics on 12/12/17.
// */
//
///*
//    A class to help us use vuforia more easily
//    Keep in mind to call the loadCipherAssets function if you wish to use this for crypto-key image detection
//    The
// */
//public class VuforiaHelper {
//    public static String LICENSE_KEY_NO_EXTERNAL_CAMERA = "Afh+Mi//////AAAAGT/WCUCZNUhEt3/AvBZOSpKBjwlgufihL3d3H5uiMfbq/1tDOM6w+dgMIdKUvVFEjNNy9zSaruPDbwX0HwjI6BEvxuWbw+UcZFcfF7i4g7peD4zSCEyZBCi59q5H/a2aTsnJVaG0WO0pPawHDuuScrMsA/QPKQGV/pZOT6rK8cW2C3bEkZpZ1qqkSM5zNeKs2OQtr8Bvl2nQiVK6mQ3ZT4fxWGb7P/iTZ4k1nEhkxI56sr5HlxmSd0WOx9i8hYDTJCASU6wwtOeUHZYigZmdRYuARS+reLJRXUylirmoU8kVvMK1p2Kf8dajEWsTuPwBec/BSaygmpqD0WkAc2B1Vmaa/1zTRfYNR3spIfjHQCYu";
//    public static String LICENSE_KEY_EXTERNAL_CAMERA = "ASTjkxr/////AAABmXsyKKnvFEZUjIMLJz2b8nmOO3tp/i6ZKChx0lsi1SX8+TvocOrfl1mDnQmCpwY0VLXKXtZ7ukuwWVj8nf+8A2ybtp9/t7j/L5rrTGqr1mM+j9W78MvFKs4VaSnpP7lETjV2VnrubiIAnhrlGB5YKUftkbOmY0xkMg0U3/KuZ1vvxrWOS26igF654pX8gXujMukiwOFSuz+Ki1iOApuiLACMRfxV5iYUtGmArO+YiGNK8p6HPmv2bnm1GQKeGr7e6Du8IyvPboOVk594ftlkokkedkXcJawqMRHWgQYUvLn9Cq4+RgjntZTcPb7Auv0jCXZyMpblZQMa0DGbns3lJNr593HQFcqVeDBjfnGY/1+Q";
//    static VuforiaLocalizer vuLoc;
//    public VuforiaTrackables targetsRoverRuckus;
//    VuforiaTrackable blueRover;
//    VuforiaTrackable redFootprint;
//    VuforiaTrackable frontCraters;
//    VuforiaTrackable backSpace;
//    private final float UPRIGHT_POST_ROTATE_IN_DEG = 270;
//    private final float HORIZONTAL_WITH_CAMERA_TO_LEFT_POST_ROTATE_IN_DEG = 180;
//    private final float WEBCAM_POST_ROTATE_IN_DEG = 0;
//
//    public VuforiaHelper(HardwareMap hw) { initVuforia(hw); }
//
//    public static VuforiaLocalizer initVuforia(HardwareMap hardwareMap) {
//        try {
////            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
//            params.vuforiaLicenseKey = LICENSE_KEY_EXTERNAL_CAMERA;
//            params.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//            params.useExtendedTracking = false;
//            vuLoc = ClassFactory.getInstance().createVuforia(params);
//            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
//            vuLoc.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
//
//        } catch (Exception e) {
//            throw new RuntimeException(e);
//        }
//        return vuLoc;
//    }
//
//    public static VuforiaLocalizer initVuforia(int camera, HardwareMap hardwareMap) {
//        switch (camera) {
//            case WEBCAM:
//                return initVuforia(hardwareMap);
//            case PHONE_CAMERA:
//                try {
//                    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
//                    params.vuforiaLicenseKey = LICENSE_KEY_NO_EXTERNAL_CAMERA;
//                    params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//                    params.cameraName = UnknownCameraNameImpl.forUnknown();
////                    params.cameraName =
//                    vuLoc = ClassFactory.getInstance().createVuforia(params);
//                    Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
//                    vuLoc.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
//                } catch (Exception e) {
//                    throw new RuntimeException(e);
//                }
//                return vuLoc;
//            default:
//                return initVuforia(hardwareMap);
//        }
//    }
//
//    public static void kill(){
//        Vuforia.deinit();
//    }
//
//    public void loadNavigationAssets(){
//        targetsRoverRuckus = vuLoc.loadTrackablesFromAsset("RoverRuckus");
//        blueRover = targetsRoverRuckus.get(0);
//        blueRover.setName("Blue-Rover");
//        redFootprint = targetsRoverRuckus.get(1);
//        redFootprint.setName("Red-Footprint");
//        frontCraters = targetsRoverRuckus.get(2);
//        frontCraters.setName("Front-Craters");
//        backSpace = targetsRoverRuckus.get(3);
//        backSpace.setName("Back-Space");
//        targetsRoverRuckus.activate();
//    }
//
//    /**
//        @param wantedWidth -- of the image to return
//        @param wantedHeight -- of the image to return
//        @return a Bitmap of what the camera sees
//     */
//    public Bitmap getImage(int wantedWidth, int wantedHeight) {
//        Image img;
//        long timeStart = System.currentTimeMillis();
//        try {
//            Log.d("Get Image", "taking image");
//            img = takeImage();
//        } catch (Exception e) {
//            Log.d("Error", e.toString());
//            throw new RuntimeException(e);
//        }
//        Log.d("VH IMG TAKE TIME", "" + (System.currentTimeMillis() - timeStart));
//
//        if(img != null) {
//            long conversionStart = System.currentTimeMillis();
//            Bitmap bmp = convertImageToBmp(img);
//            Log.d("VH IMG Convert", "" + (System.currentTimeMillis() - conversionStart));
//            long copyStart = System.currentTimeMillis();
//            Bitmap orig = bmp.copy(Bitmap.Config.ARGB_8888,true);
//            Log.d("VH IMG ORIG","Height: " + orig.getHeight() + " Width: " + orig.getWidth());
//            Log.d("VH IMG CPY", "" + (System.currentTimeMillis() - copyStart));
//            long scaleStart = System.currentTimeMillis();
//            Matrix matrix = new Matrix();
//            Bitmap scaledBitmap = Bitmap.createScaledBitmap(orig,wantedWidth,wantedHeight,true);
//            matrix.postRotate(WEBCAM_POST_ROTATE_IN_DEG);
//            Log.d("VH IMG Scale", "" + (System.currentTimeMillis() - scaleStart));
//            long rotationStart = System.currentTimeMillis();
//            Bitmap rotatedBitmap = Bitmap.createBitmap(scaledBitmap , 0, 0, scaledBitmap .getWidth(), scaledBitmap .getHeight(), matrix, true);
//            rotatedBitmap = Bitmap.createScaledBitmap(rotatedBitmap,wantedWidth,wantedHeight,true);
//            Log.d("VH IMG Rotation", "" + (System.currentTimeMillis() - rotationStart));
//            return rotatedBitmap;
//        }
//        return null;
//    }
//
//    /*
//        convertImageToBmp
//        requires and Image of what to convert
//        returns the Bitmap of the Image
//     */
//    private Bitmap convertImageToBmp(Image img){
//        Bitmap bmp = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
//        bmp.copyPixelsFromBuffer(img.getPixels());
//        return bmp;
//    }
//
//
//    /*
//        takeImage()
//        this function gets vuforia to return an Image of what it sees
//     */
//    private Image takeImage() throws InterruptedException {
//        Image img = null;
//        VuforiaLocalizer.CloseableFrame frame = vuLoc.getFrameQueue().take(); //takes the frame at the head of the queue
//
//        long numImages = frame.getNumImages();
//        Log.d("Take Image", "getting frames");
//        for (int i = 0; i < numImages; i++) {
//            Log.d("Format","" + frame.getImage(i).getFormat());
//            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                img = frame.getImage(i);
//                break;
//            }
//        }
//        return img;
//    }
//
//
//    /*
//        saveBMP
//        requires a Bitmap of what to save to the local storage
//
//        saves a BMP to the phone, you can find it in the root directory
//     */
//    public void saveBMP(Bitmap bmp){
//
//        FileOutputStream out = null;
//        try {
//
//            File yourFile = new File(Environment.getExternalStorageDirectory().toString() + "/robot" + System.currentTimeMillis() + ".png");
//            yourFile.createNewFile(); // if file already exists will do nothing
//            Log.d("Save BMP", "created file");
//            out = new FileOutputStream(Environment.getExternalStorageDirectory().toString() + "/robot" + System.currentTimeMillis() + ".png",false);
//            Log.d("Saving",out.toString());
//            bmp.compress(Bitmap.CompressFormat.PNG, 10, out); // bmp is your Bitmap instance
//            // PNG is a lossless format, the compression factor (100) is ignored
//        } catch (Exception e) {
//            e.printStackTrace();
//        } finally {
//            try {
//                if (out != null) {
//                    out.close();
//                }
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//
//}
