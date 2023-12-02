package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class AutoRedFront extends LinearOpMode {
    /* CONFIG */
    private final boolean goUnderLeftTruss = false;
    private final double delayAtTrussSeconds = 0;
    private final boolean parkLeft = false;

    /* GLOBAL VARIABLES */
    CenterStageRobot robot;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //1 = left, 2 = center, 3 = right
    private int placementPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-36,-60),Math.PI/2), this);
        tfod = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        /* POSITION IDENTIFICATION */
        while(!isStarted()){
            telemetry.addLine("RED, FRONTSTAGE - Ready");
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.size() == 0){
                telemetry.addLine("No Objects Detected. Using last known detection.");
            }else if(currentRecognitions.size() > 1){
                telemetry.addLine("***Multiple Objects Detected***");
            }
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                //TODO: Find binding area for the 3 placements

            }
            telemetry.addData("Placement Position", placementPosition);
            telemetry.update();
        }
        visionPortal.close();
        /* PIXEL ON SPIKE STRIP */
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36,-36))
                            .strafeTo(new Vector2d(-48, -36))
                            .build());
            //TODO: place the pixel down
        }else if(placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36, -24))
                            .build());
            //TODO: place the pixel down
        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36,-36))
                            .strafeTo(new Vector2d(-24, -36))
                            .build());
            //TODO: place the pixel down
        }
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeTo(new Vector2d(-36, -36))
                        .build());
        /* DRIVE TO BACKSTAGE */
        if(goUnderLeftTruss){
            //Under the Left Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-12,-36), 0)
                            .waitSeconds(delayAtTrussSeconds)
                            //TODO: raise slides after leaving truss
                            .strafeTo(new Vector2d(48, -36))
                            .build());
        }else{
            //Under the Right Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .splineTo(new Vector2d(-24,-60), Math.PI/2)
                            .strafeTo(new Vector2d(-12,-60))
                            .waitSeconds(delayAtTrussSeconds)
                            //TODO: raise slides after leaving truss
                            .splineTo(new Vector2d(48, -36),0)
                            .build());
        }

        /* PLACE ON BACKDROP */
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -30))
                            .build());
            //TODO: place the pixel down
        }else if(placementPosition == 2) {
            //Center

            //TODO: place the pixel down
        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -42))
                            .build());
            //TODO: place the pixel down
        }

        /* PARK */
        //TODO: lower slides
        if(parkLeft){
            //Park on Left Side
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -12))
                            .build());
        }else{
            //Park on Right Side
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -60))
                            .build());
        }
    }
}
