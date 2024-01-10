package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
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
        Imgproc.line(frame, new Point(CenterStageRobot.leftPlacementLowerBound, 0), new Point(CenterStageRobot.leftPlacementLowerBound,height), new Scalar(255,0,0), 3);
        Imgproc.line(frame, new Point(CenterStageRobot.leftPlacementUpperBound, 0), new Point(CenterStageRobot.leftPlacementUpperBound,height), new Scalar(255,0,0), 3);
        Imgproc.putText(frame, "L", new Point(CenterStageRobot.leftPlacementUpperBound, 50),1,15, new Scalar(255,0,0));
        //CENTER
        Imgproc.line(frame, new Point(CenterStageRobot.centerPlacementLowerBound, 0), new Point(CenterStageRobot.centerPlacementLowerBound,height), new Scalar(255,0,0), 3);
        Imgproc.line(frame, new Point(CenterStageRobot.centerPlacementUpperBound, 0), new Point(CenterStageRobot.centerPlacementUpperBound,height), new Scalar(255,0,0), 3);
        Imgproc.putText(frame, "C", new Point(CenterStageRobot.centerPlacementUpperBound-5, 50),1,15, new Scalar(255,0,0));
        //RIGHT
//        Imgproc.line(frame, new Point(CenterStageRobot.rightPlacementLowerBound, 0), new Point(CenterStageRobot.rightPlacementLowerBound,height), new Scalar(255,0,0), 3);
//        Imgproc.line(frame, new Point(CenterStageRobot.rightPlacementUpperBound, 0), new Point(CenterStageRobot.rightPlacementUpperBound,height), new Scalar(255,0,0), 3);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
