package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class MachineVision {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    private int placementPosition = 1;
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
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
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
                    placementPosition = 3;
                    opMode.telemetry.addLine("No Objects Detected. Assuming Right.");
                }else {
                    opMode.telemetry.addLine("No Objects Detected. Waiting 60 frames.");
                }
            }else if(currentRecognitions.size() > 1){
                opMode.telemetry.addLine("***Multiple Objects Detected***");
            }
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
//                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if(recognition.getLabel().equals("blueElement") || recognition.getLabel().equals("redElement")){
                    if(x > CenterStageRobot.leftPlacementLowerBound && x < CenterStageRobot.leftPlacementUpperBound){
                        framesWithoutDetection = 0;
                        //LEFT
                        placementPosition = 1;
                    }else if(x > CenterStageRobot.centerPlacementLowerBound && x < CenterStageRobot.centerPlacementUpperBound){
                        framesWithoutDetection = 0;
                        //CENTER
                        placementPosition = 2;
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
