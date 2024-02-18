/*
Copyright (c) 2019 Ryan Brott

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Ryan Brott nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.external.stream;

import android.graphics.Bitmap;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

import java.io.ByteArrayOutputStream;

public class CameraStreamServer {

    private static final CameraStreamServer INSTANCE = new CameraStreamServer();
    public static CameraStreamServer getInstance() {
        return INSTANCE;
    }

    private CameraStreamServer() {

    }

    public synchronized void setSource(@Nullable CameraStreamSource source) {
    }

    public int getJpegQuality() {
        return 1;
    }

    public void setJpegQuality(int quality) {
    }

//    public CallbackResult handleRequestFrame() {
//        if (source != null) {
//            synchronized (this) {
//                source.getFrameBitmap(Continuation.createTrivial(new Consumer<Bitmap>() {
//                    @Override
//                    public void accept(Bitmap bitmap) {
//                        sendFrame(bitmap);
//                    }
//                }));
//            }
//        }
//
//        return CallbackResult.HANDLED;
//    }

    private void sendFrame(Bitmap bitmap) {
    }

}
