package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
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

@Autonomous(name="Red - Frontstage")
public class AutoRedFront extends LinearOpMode {
    /* CONFIG */
    private final boolean goUnderLeftTruss = false;
    private final double delayAtTrussSeconds = 0;
    private final boolean parkLeft = false;

    /* GLOBAL VARIABLES */
    CenterStageRobot robot;
    MachineVision machineVision;

    //1 = left, 2 = center, 3 = right
    private int placementPosition = 1;

    //frame counter before reverting
    private int framesWithoutDetection = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-36,-62),3*Math.PI/2), this);
        machineVision = new MachineVision(hardwareMap, this);
        /* POSITION IDENTIFICATION */
        //Go find the position
        placementPosition = machineVision.run();
        robot.garageDoorServo.setPosition(1);
        sleep(200);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);


        if (placementPosition == 1) {
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, -38), 3*Math.PI / 2)
                            .strafeTo(new Vector2d(-40, -38))
                            .build());
        } else if (placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, -32), Math.PI)
                            .build());
        } else if (placementPosition == 3) {
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, -38),Math.PI / 2)
                            .strafeTo(new Vector2d(-30, -38))
                            .build());
        }
        robot.purplePixelServo.setPosition(0);
        sleep(600); /* wait for pixel to fall */
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .lineToX(-36)
                        .strafeTo(new Vector2d(-36, -55))
                        .build());
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeTo(new Vector2d(-36, -36))
                        .build());
        /* DRIVE TO BACKSTAGE */
        if(goUnderLeftTruss){
            //Under the Left (Nearest Stage Door) Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-12,-36), 0)
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(48, -36))
                            .afterDisp(48, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .build());
        }else{
            //Under the Right (Nearest Wall) Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .splineTo(new Vector2d(-24,-60), Math.PI/2)
                            .strafeTo(new Vector2d(-12,-60))
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(48, -60))
                            .afterDisp(64, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .strafeTo(new Vector2d(48, -36))
                            .build());
        }

       /*Place on Backboard*/
        robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
        robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
        if (placementPosition == 1) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53, -28))
                            .build());
        } else if (placementPosition == 2) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53, -41))
                            .build());
        } else if (placementPosition == 3) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53, -49))
                            .build());
        }

        sleep(500);
        robot.clawServo.setPosition(1); /* place the pixel */
        sleep(500); /* wait for pixel to drop */
        robot.clawServo.setPosition(0);
        //Park in red backstage
        if (parkLeft) {
            //Park on Left Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .lineToX(45)
                            .strafeTo(new Vector2d(50, -12))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        } else {
            //Park on Right Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .lineToX(45)
                            .strafeTo(new Vector2d(50, -60))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }
        //Reset

        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        robot.wristServo.setPosition(CenterStageRobot.wristCollapsePosition);
        sleep(1000);
    }

}
