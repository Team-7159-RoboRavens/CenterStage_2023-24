package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.THEArmBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.KrishDriveBM;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@TeleOp(name="Button Map Test: Krish Drive & Common Arm")
public class BMTestTeleop2 extends OpMode {
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
        buttonMap = new KrishDriveBM();
        slidesButtonMap = new THEArmBM();
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
