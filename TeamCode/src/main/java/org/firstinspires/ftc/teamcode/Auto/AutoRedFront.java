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
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-36,-60),3*Math.PI/2), this);
        machineVision = new MachineVision(hardwareMap, this);
        /* POSITION IDENTIFICATION */
        placementPosition = machineVision.run();
        robot.garageDoorServo.setPosition(1);
        sleep(200);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        /* PIXEL ON SPIKE STRIP */
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-36,-36))
                            .strafeToLinearHeading(new Vector2d(-41, -36), 3*Math.PI/2)
                            .build());
            //TODO: find number
            robot.purplePixelServo.setPosition(0);
        }else if(placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-43, -24), Math.PI/2)
                            .build());
            robot.purplePixelServo.setPosition(0);
        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36,-36), Math.PI/2)
                            .strafeTo(new Vector2d(-31, -36))
                            .build());
            robot.purplePixelServo.setPosition(0);
        }
        sleep(500); /* wait for pixel to fall */
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

        /* PLACE ON BACKDROP */
        robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
        robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
        if(placementPosition == 1){
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -30))
                            .build());

        }else if(placementPosition == 2) {
            //Center
            sleep(1500); /* wait for arm */
        }else if(placementPosition == 3){
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -42))
                            .build());

        }
        robot.clawServo.setPosition(1); /* place the pixel */
        sleep(300); /* wait for pixel to drop */
        //Reset
        robot.clawServo.setPosition(0);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        robot.wristServo.setPosition(CenterStageRobot.wristCollapsePosition);
        /* PARK */
        if(parkLeft){
            //Park on Left Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -12))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }else{
            //Park on Right Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(48, -60))
                            .build(),
                    robot.setSlideHeightAction(0)
            ));
        }
    }
}
