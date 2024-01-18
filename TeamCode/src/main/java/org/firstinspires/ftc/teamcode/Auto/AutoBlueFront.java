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
    MachineVision machineVision;

    //1 = left, 2 = center, 3 = right
    private int placementPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-33,62),Math.PI/2), this);
        machineVision = new MachineVision(hardwareMap, this);
        /* POSITION IDENTIFICATION */
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        robot.garageDoorServo.setPosition(1);
        sleep(200);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        //Go find the position
        telemetry.addLine("Checking Machine Vision");
        telemetry.update();
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeToLinearHeading(new Vector2d(-30, 56), Math.PI / 2)
                        .build());
        placementPosition = machineVision.run();
        /* PIXEL ON SPIKE STRIP */
        ///eeeee
        if (placementPosition == 1) {
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36, 38))
                            .strafeToLinearHeading(new Vector2d(-30, 38), Math.PI / 2)
                            .build());
        } else if (placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, 32), 0)
                            .build());
        } else if (placementPosition == 3) {
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, 38), 3*Math.PI / 2)
                            .strafeTo(new Vector2d(-40, 38))
                            .build());
        }
        robot.purplePixelServo.setPosition(0);
        sleep(500); /* wait for pixel to fall */
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeTo(new Vector2d(-36, 55))
                        .build());
//        Actions.runBlocking(
//                robot.actionBuilder(robot.pose)
//                        .strafeTo(new Vector2d(-36, 36))
//                        .build());
//        /* DRIVE TO BACKSTAGE */
//        if(goUnderLeftTruss){
//            //Under the Left (Nearest Wall) Truss
//            Actions.runBlocking(
//                    robot.actionBuilder(robot.pose)
//                            .splineTo(new Vector2d(-24,60), Math.PI/2)
//                            .strafeTo(new Vector2d(-12,60))
//                            .waitSeconds(delayAtTrussSeconds)
//                            .strafeTo(new Vector2d(48, 60))
//                            .afterDisp(64, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
//                            .strafeTo(new Vector2d(48, 36))
//                            .build());
//        }else{
//            //Under the Right (Nearest Stage Door) Truss
//            Actions.runBlocking(
//                    robot.actionBuilder(robot.pose)
//                            .strafeToLinearHeading(new Vector2d(-12,36), 0)
//                            .waitSeconds(delayAtTrussSeconds)
//                            .strafeTo(new Vector2d(48, 36))
//                            .afterDisp(48, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
//                            .build());
//        }
//
//        /* PLACE ON BACKDROP */
//        robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
//        robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
//        Actions.runBlocking(
//                robot.actionBuilder(robot.pose)
//                        .strafeTo(new Vector2d(50, 48 - (6 * placementPosition))) // 48 is the upper bound of the board's tile's y position and placement positions are 6in apart
//                        .build());
//        robot.clawServo.setPosition(1); /* place the pixel */
//        sleep(300); /* wait for pixel to drop */
//        //Reset
//        robot.clawServo.setPosition(0);
//        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
//        robot.wristServo.setPosition(CenterStageRobot.wristCollapsePosition);
//
//        /* PARK */
//        if(parkLeft){
//            //Park on Left Side
//            Actions.runBlocking(new ParallelAction(
//                    robot.actionBuilder(robot.pose)
//                            .strafeTo(new Vector2d(48, 60))
//                            .build(),
//                    robot.setSlideHeightAction(0)
//            ));
//        }else{
//            //Park on Right Side
//            Actions.runBlocking(new ParallelAction(
//                    robot.actionBuilder(robot.pose)
//                            .strafeTo(new Vector2d(48, 12))
//                            .build(),
//                    robot.setSlideHeightAction(0)
//            ));
//        }
    }
}
