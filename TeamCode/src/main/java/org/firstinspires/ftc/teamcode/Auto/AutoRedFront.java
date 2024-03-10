package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

import java.util.Vector;

@Autonomous(name="Red - Frontstage")
public class AutoRedFront extends LinearOpMode {
    /* CONFIG */
    //True: Goes under the red side of the stage door and enters from left
    //False: Goes under the closer-to-wall side of the truss and enters from right
    private final boolean goUnderStageDoor = true;
    private final double delayAtTrussSeconds = 0.1;
    private final boolean parkLeft = true;

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
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(-39, -62), 3 * Math.PI / 2), this);
        machineVision = new MachineVision(hardwareMap, this);
        /* POSITION IDENTIFICATION */
        //Go find the position
        placementPosition = machineVision.run("Red, Frontstage - READY");
        robot.garageDoorServo.setPosition(1);
        sleep(200);
        robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
        //Purple Pixel
        if (placementPosition == 1) {
            //Left
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-47,  -46.5))
                            .turnTo(Math.PI)
                            .strafeTo(new Vector2d(-47, -22))
                            .strafeTo(new Vector2d(-48, -26))
                            .build());
        } else if (placementPosition == 2) {
            //Center
            if(goUnderStageDoor){
                //Prepare for Stage Door - go past
                Actions.runBlocking(
                        robot.actionBuilder(robot.pose)
                                .strafeToLinearHeading(new Vector2d(-36, -36), Math.PI)
                                .strafeTo(new Vector2d(-36, -19))
                                .build());
            }else{
                Actions.runBlocking(
                        robot.actionBuilder(robot.pose)
                                .strafeTo(new Vector2d(-36, -60))
                                .strafeToLinearHeading(new Vector2d(-36, -32), Math.PI)
                                .build());
            }

        } else if (placementPosition == 3) {
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36, -36.5), 3*Math.PI / 2)
                            .strafeTo(new Vector2d(-32, -36.5))
                            .build());
        }
        robot.purplePixelServo2.setPosition(1);
        sleep(700); /* wait for pixel to fall */
        if(placementPosition == 3){
            //Drive back to the center if we were on an edge
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(-37, -36))
                            .turnTo(Math.PI)
                            .build());
        }
        /* DRIVE TO BACKSTAGE */
        if(goUnderStageDoor){
            //Under the Stage Door
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36,-11), Math.PI)
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(20, -12))
                            .strafeTo(new Vector2d(48, -12))
                            .strafeTo(new Vector2d(48, -36))
                            .afterDisp(60, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .build());
        }else{
            //Under the Nearest Wall Truss
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeToLinearHeading(new Vector2d(-36,-60), Math.PI)
                            .strafeTo(new Vector2d(-12,-60))
                            .waitSeconds(delayAtTrussSeconds)
                            .strafeTo(new Vector2d(48, -60))
                            .afterDisp(64, robot.setSlideHeightAction(CenterStageRobot.slidesRaisePosition))
                            .strafeTo(new Vector2d(48, -36))
                            .build());
        }

       /*Place on Backboard*/
        if (placementPosition == 1) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53.5, -30))
                            .build());
        } else if (placementPosition == 2) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53.5, -35.5))
                            .build());
        } else if (placementPosition == 3) {
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(53.5, -46))
                            .build());
        }
        robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
        robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
        sleep(2500);
        robot.clawServo.setPosition(1); /* place the pixel */
        sleep(500); /* wait for pixel to drop */
        robot.clawServo.setPosition(0);
        //Park in red backstage
        if (parkLeft) {
            //Park on Left Side
            Actions.runBlocking(new ParallelAction(
                    robot.actionBuilder(robot.pose)
                            .lineToX(45)
                            .strafeTo(new Vector2d(50, -14))
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
        sleep(1500);
    }

}
