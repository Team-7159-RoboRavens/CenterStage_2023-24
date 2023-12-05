package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.EitanNadavDriveBM;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@TeleOp(name="Button Map Test: Eitan/Nadav Drive, Ariana Arm")
public class BMTestTeleop extends OpMode {
    //Global Variables
    CenterStageRobot robot;

    //Button Maps
    AbstractButtonMap buttonMap;
    AbstractButtonMap slidesButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new CenterStageRobot(hardwareMap, new Pose2d(0,0,0), this);
        buttonMap = new EitanNadavDriveBM();
//        slidesButtonMap = new ArianaArmBm();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        buttonMap.loop(robot, this);
        slidesButtonMap.loop(robot, this);
        telemetry.update();
    }
}
