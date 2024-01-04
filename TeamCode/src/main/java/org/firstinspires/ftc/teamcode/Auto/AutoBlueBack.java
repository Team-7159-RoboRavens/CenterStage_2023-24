package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="Blue - Backstage")
public class AutoBlueBack extends LinearOpMode {
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
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                    .strafeTo(new Vector2d(12,24))
                    .strafeToLinearHeading(new Vector2d(48, 48-(6*placementPosition)), 0) // 48 is the upper bound of the board's tile's y position and placement positions are 6in apart
                    .build());
        //TODO: find magic number
        robot.purplePixelServo.setPosition(0);

        //Park in blue backstage
        Actions.runBlocking(
            robot.actionBuilder(robot.pose)
                .lineToY(12)
                .build()
        );
    }
}
