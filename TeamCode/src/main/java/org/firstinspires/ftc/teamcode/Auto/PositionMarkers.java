package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//Puts lines on the camera output for alignment purposes.
public class PositionMarkers implements VisionProcessor {
    private int height;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //LEFT
        Imgproc.line(frame, new Point(35, 0), new Point(25,height), new Scalar(,0,0), 3);
        Imgproc.line(frame, new Point(65, 0), new Point(30,height), new Scalar(255,0,0), 3);
        //CENTER
        Imgproc.line(frame, new Point(320, 0), new Point(200,height), new Scalar(255,0,0), 3);
        Imgproc.line(frame, new Point(350, 0), new Point(230,height), new Scalar(255,0,0), 3);
        //RIGHT
        Imgproc.line(frame, new Point(500, 0), new Point(400,height), new Scalar(255,0,0), 3);
        Imgproc.line(frame, new Point(520, 0), new Point(405,height), new Scalar(255,0,0), 3);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
