package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@TeleOp
public class ArmTuner extends OpMode {
    CenterStageRobot robot;
    ElapsedTime et;
    double servoTime;
    @Override
    public void init() {
        robot = new CenterStageRobot(hardwareMap, new Pose2d(0,0,0), this);
        et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        servoTime = 0;
    }

    @Override
    public void loop() {
        if(et.time()-servoTime > 150){
            if(gamepad1.x){
                robot.airplaneServo.setPosition(robot.airplaneServo.getPosition() + 0.05);
                servoTime = et.time();
            }else if(gamepad1.b){
                robot.airplaneServo.setPosition(robot.airplaneServo.getPosition() - 0.05);
                servoTime = et.time();
            }
            if(gamepad1.y){
                robot.wristServo.setPosition(robot.wristServo.getPosition()+0.05);
                servoTime = et.time();
            } else if (gamepad1.a) {
                robot.wristServo.setPosition(robot.wristServo.getPosition()-0.05);
                servoTime = et.time();
            }
            if(gamepad1.dpad_up){
                robot.elbowServo.setPosition(robot.elbowServo.getPosition()+0.05);
                servoTime = et.time();
            } else if (gamepad1.dpad_down) {
                robot.elbowServo.setPosition(robot.elbowServo.getPosition()-0.05);
                servoTime = et.time();
            }
            if(gamepad1.left_bumper){
                robot.garageDoorServo.setPosition(robot.garageDoorServo.getPosition()+0.05);
                servoTime = et.time();
            } else if (gamepad1.right_bumper) {
                robot.garageDoorServo.setPosition(robot.garageDoorServo.getPosition()-0.05);
                servoTime = et.time();
            }
            if(gamepad1.dpad_left){
                robot.purplePixelServo.setPosition(robot.purplePixelServo.getPosition()+0.05);
                servoTime = et.time();
            } else if (gamepad1.dpad_right) {
                robot.purplePixelServo.setPosition(robot.purplePixelServo.getPosition()-0.05);
                servoTime = et.time();
            }
        }

        telemetry.addLine("Test Controls and Positions (GP1):");
        telemetry.addData("Airplane Servo (X=inc, B=dec)", robot.airplaneServo.getPosition());
        telemetry.addData("Wrist Servo (Y=inc, A=dec)", robot.wristServo.getPosition());
        telemetry.addData("Elbow Servo (DPup=inc, DPdn=dec)", robot.elbowServo.getPosition());
        telemetry.addData("Purple Pixel Servo (DPleft=inc, DPright=dec)", robot.purplePixelServo.getPosition());
        telemetry.addData("Garage Door Servo (LB=inc, RB=dec)", robot.garageDoorServo.getPosition());
        telemetry.update();
    }
}
