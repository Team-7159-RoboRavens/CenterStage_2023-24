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

@Autonomous(name = "Red - Backstage")
public class AutoRedBack extends LinearOpMode {
    public final boolean parkLeft = true;

    CenterStageRobot robot;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private int placementPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(12, -60), 3*Math.PI / 2), this);
        String[] labels = {"redElement", "blueElement"};
        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("CustomElements.tflite")
                .setModelLabels(labels)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        /* POSITION IDENTIFICATION */
        while (!isStarted()) {
            telemetry.addLine("RED, BACKSTAGE - Ready");
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions.size() == 0) {
                telemetry.addLine("No Objects Detected. Using last known detection.");
            } else if (currentRecognitions.size() > 1) {
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
        if (placementPosition == 1) {
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(12, -36), 3 * Math.PI / 2)
                            .strafeTo(new Vector2d(7, -36))
                            .build());
        } else if (placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(19, -24), 3 * Math.PI / 2)
                            .build());
        } else if (placementPosition == 3) {
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(12, -36))
                            .strafeToLinearHeading(new Vector2d(17, -36), Math.PI / 2)
                            .build());
        }
        robot.purplePixelServo.setPosition(0);
        sleep(500); /* wait for pixel to fall */
        /*Place on Backboard*/
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .afterTime(1, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                        .strafeToLinearHeading(new Vector2d(48, -36), Math.PI)
                        .strafeTo(new Vector2d(48, -24 - (6 * placementPosition))) // 48 is the upper bound of the board's tile's y position and placement positions are 6in apart
                        .build());
        robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
        robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
        sleep(1000);
        robot.clawServo.setPosition(1); /* place the pixel */
        sleep(300); /* wait for pixel to drop */
        //Reset
        robot.clawServo.setPosition(0);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        robot.wristServo.setPosition(CenterStageRobot.wristCollapsePosition);

        //Park in blue backstage
        if(parkLeft){
            //Park on Left Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -60))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }else{
            //Park on Right Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -12))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }
    }
}
