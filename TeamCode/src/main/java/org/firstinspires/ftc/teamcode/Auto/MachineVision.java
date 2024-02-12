package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class MachineVision {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    public static int placementPosition = 1;
    public static int defaultPlacementPosition = 3;
    private int framesWithoutDetection = 0;

    public MachineVision(HardwareMap hm, LinearOpMode om){
        hardwareMap = hm;
        opMode = om;
        String[] labels = {"blueElement", "redElement"};
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("CustomElements.tflite")
                .setModelLabels(labels)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod ,new PositionMarkers());
//        visionPortal.stopStreaming();
    }

    public int run() {
//        visionPortal.resumeStreaming();
        while(!opMode.isStarted()){
            opMode.telemetry.addLine("Ready");
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.size() == 0){
                framesWithoutDetection++;
                //If we haven't detected anything for 60 frames, assume right
                if(framesWithoutDetection > 60){
                    placementPosition = /*3*/ defaultPlacementPosition;
                    opMode.telemetry.addLine("No Objects Detected. Assuming Right.");
                }else {
                    opMode.telemetry.addLine("No Objects Detected. Waiting 60 frames.");
                }
            }else if(currentRecognitions.size() > 1){
                opMode.telemetry.addLine("***Multiple Objects Detected***");
            }
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                if(recognition.getWidth() > 300){
                    opMode.telemetry.addLine(recognition.getLabel() + " at " + x + ": too wide! width: "+recognition.getWidth());
                    continue;
                }
                if(recognition.getLabel().equals("blueElement") || recognition.getLabel().equals("redElement")){

                    if(x > CenterStageRobot.leftPlacementLowerBound && x < CenterStageRobot.leftPlacementUpperBound){
                        opMode.telemetry.addLine(recognition.getLabel() + " at " + x + " in range for LEFT.");
                        framesWithoutDetection = 0;
                        //LEFT
                        placementPosition = 1;
                    }else if(x > CenterStageRobot.centerPlacementLowerBound && x < CenterStageRobot.centerPlacementUpperBound){
                        opMode.telemetry.addLine(recognition.getLabel() + " at " + x + " in range for CENTER.");
                        framesWithoutDetection = 0;
                        //CENTER
                        placementPosition = 2;
                    }else{
                        opMode.telemetry.addLine(recognition.getLabel() + " at " + x + ": out of range!");
                    }
                }
            }
            opMode.telemetry.addData("Placement Position", placementPosition);
            opMode.telemetry.update();
        }
        visionPortal.close();
        return placementPosition;
    }
}