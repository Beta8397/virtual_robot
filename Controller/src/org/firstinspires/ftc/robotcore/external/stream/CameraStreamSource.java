package org.firstinspires.ftc.robotcore.external.stream;

import android.graphics.Bitmap;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

public interface CameraStreamSource {
    void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation);
}
