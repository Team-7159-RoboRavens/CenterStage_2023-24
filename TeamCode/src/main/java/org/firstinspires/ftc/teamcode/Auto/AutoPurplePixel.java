package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Autonomous(name = "Auto PURPLE PIXEL ONLY")
public class AutoPurplePixel extends LinearOpMode {
    CenterStageRobot robot;
    MachineVision machineVision;

    private int placementPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        /* INITIALIZATION */
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(9, -62), 3 * Math.PI / 2), this);
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
                            .strafeTo(new Vector2d(12, -60))
                            .strafeToLinearHeading(new Vector2d(12, -36), 3 * Math.PI / 2)
                            .strafeTo(new Vector2d(7, -36))
                            .build());
        } else if (placementPosition == 2) {
            //Center
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(12, -60))
                            .strafeToLinearHeading(new Vector2d(12, -32), Math.PI)
                            .build());
        } else if (placementPosition == 3) {
            //Right
            Actions.runBlocking(
                    robot.actionBuilder(robot.pose)
                            .strafeTo(new Vector2d(12, -60))
                            .strafeTo(new Vector2d(12, -36))
                            .strafeToLinearHeading(new Vector2d(17, -36), Math.PI / 2)
                            .build());
        }
        robot.purplePixelServo.setPosition(0);
        sleep(1000);
        Actions.runBlocking(
                robot.actionBuilder(robot.pose)
                        .strafeTo(new Vector2d(12, -36))
                        .build());
    }
}
