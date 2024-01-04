package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="Blue - Frontstage")
public class AutoBlueFront extends LinearOpMode {
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

    //frame counter before reverting
    private int framesWithoutDetection = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-36,60),Math.PI/2), this);
        tfod = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod, new PositionMarkers());

        /* POSITION IDENTIFICATION */
        while(!isStarted()){
            telemetry.addLine("BLUE, FRONTSTAGE - Ready");
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.size() == 0){
                framesWithoutDetection++;
                //If we haven't detected anything for 60 frames, assume right
                if(framesWithoutDetection > 60){
                    placementPosition = 3;
                    telemetry.addLine("No Objects Detected. Assuming Right");
                }else {
                    telemetry.addLine("No Objects Detected. Waiting 60 frames.");
                }
            }else if(currentRecognitions.size() > 1){
                telemetry.addLine("***Multiple Objects Detected***");
            }
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
//                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                //TODO: change to whatever game element is now
                if(recognition.getLabel().equals("Pixel")){
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
            telemetry.addData("Placement Position", placementPosition);
            telemetry.update();
        }
        visionPortal.close();
        /* PIXEL ON SPIKE STRIP */
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36,36))
                            .strafeToLinearHeading(new Vector2d(-31, 36), Math.PI/2)
                            .build());
            //TODO: find number
            robot.purplePixelServo.setPosition(0);
        }else if(placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-29, 24), 3*Math.PI/2)
                            .build());
            //TODO: find number
            robot.purplePixelServo.setPosition(0);
        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36,36), 3*Math.PI/2)
                            .strafeTo(new Vector2d(-41, 36))
                            .build());
            //TODO: find number
            robot.purplePixelServo.setPosition(0);
        }
        sleep(500); /* wait for pixel to fall */
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeTo(new Vector2d(-36, 36))
                        .build());
        /* DRIVE TO BACKSTAGE */
        if(goUnderLeftTruss){
            //Under the Left (Nearest Wall) Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .splineTo(new Vector2d(-24,60), Math.PI/2)
                            .strafeTo(new Vector2d(-12,60))
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(48, 60))
                            .afterDisp(64, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .strafeTo(new Vector2d(48, 36))
                            .build());
        }else{
            //Under the Right (Nearest Stage Door) Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-12,36), 0)
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(48, 36))
                            .afterDisp(48, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .build());
        }

        /* PLACE ON BACKDROP */
        //TODO: find numbers
        robot.elbowServo.setPosition(0.25);
        robot.wristServo.setPosition(0.8);
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, 42))
                            .build());
        }else if(placementPosition == 2) {
            //Center

        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, 30))
                            .build());
        }
        //TODO: find numbers
        robot.clawServo.setPosition(1); /* place the pixel */
        sleep(300); /* wait for pixel to drop */
        //Reset
        robot.clawServo.setPosition(0);
        robot.elbowServo.setPosition(0.95);
        robot.wristServo.setPosition(1);

        /* PARK */
        //TODO: reset the positions of the servos
        if(parkLeft){
            //Park on Left Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, 60))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }else{
            //Park on Right Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, 12))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }
    }
}
