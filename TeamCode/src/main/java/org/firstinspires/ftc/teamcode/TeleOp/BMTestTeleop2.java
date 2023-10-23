package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.EitanNadavDriveBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.KrishDriveBM;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@TeleOp(name="Button Map Test 2 (Krish)")
public class BMTestTeleop2 extends OpMode {
    //Global Variables
    CenterStageRobot robot;

    //Button Maps
    AbstractButtonMap buttonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new CenterStageRobot(hardwareMap, new Pose2d(0,0,0), this);
        buttonMap = new KrishDriveBM();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        buttonMap.loop(robot, this);
        telemetry.update();
    }
}
