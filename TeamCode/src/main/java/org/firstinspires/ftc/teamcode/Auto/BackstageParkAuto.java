package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Autonomous(name="Backstage PARKING ONLY")
public class BackstageParkAuto extends LinearOpMode {

    public CenterStageRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CenterStageRobot(hardwareMap, new Pose2d(new Vector2d(0,0),0), this);
        waitForStart();
        sleep(23000);
        robot.setAllMotorPowers(0.5);
        sleep(2000);
        robot.setAllMotorPowers(0);

    }
}
