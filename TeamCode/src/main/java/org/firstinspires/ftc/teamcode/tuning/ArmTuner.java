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
        robot.airplaneServo.scaleRange(0,1);
        robot.wristServo.scaleRange(0,1);
        robot.elbowServo.scaleRange(0,1);
        robot.garageDoorServo.scaleRange(0,1);
        robot.clawServo.scaleRange(0,1);
        telemetry.addLine("ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(et.time()-servoTime > 150){
            if(gamepad1.x){
                robot.airplaneServo.setPosition(robot.airplaneServo.getPosition() + 0.02);
                servoTime = et.time();
            }else if(gamepad1.b){
                robot.airplaneServo.setPosition(robot.airplaneServo.getPosition() - 0.02);
                servoTime = et.time();
            }
            if(gamepad1.y){
                robot.wristServo.setPosition(robot.wristServo.getPosition()+0.02);
                servoTime = et.time();
            } else if (gamepad1.a) {
                robot.wristServo.setPosition(robot.wristServo.getPosition()-0.02);
                servoTime = et.time();
            }
            if(gamepad1.dpad_up){
                robot.elbowServo.setPosition(robot.elbowServo.getPosition()+0.02);
                servoTime = et.time();
            } else if (gamepad1.dpad_down) {
                robot.elbowServo.setPosition(robot.elbowServo.getPosition()-0.02);
                servoTime = et.time();
            }
            if(gamepad1.left_bumper){
                robot.garageDoorServo.setPosition(robot.garageDoorServo.getPosition()+0.02);
                servoTime = et.time();
            } else if (gamepad1.right_bumper) {
                robot.garageDoorServo.setPosition(robot.garageDoorServo.getPosition()-0.02);
                servoTime = et.time();
            }
            if(gamepad1.dpad_left){
                robot.clawServo.setPosition(robot.clawServo.getPosition()+0.02);
                servoTime = et.time();
            } else if (gamepad1.dpad_right) {
                robot.clawServo.setPosition(robot.clawServo.getPosition()-0.02);
                servoTime = et.time();
            }
        }

        if (gamepad2.right_trigger > 0.1) {
            if (robot.linearSlidesMotor1.getCurrentPosition() < -5 || robot.linearSlidesMotor2.getCurrentPosition() < -5) {
                telemetry.addData("LS Direction", "INHIBIT DOWN");
                robot.linearSlidesMotor1.setPower(0);
                robot.linearSlidesMotor2.setPower(0);
            } else {
                telemetry.addData("LS Direction", "DOWN");
                robot.linearSlidesMotor1.setPower(-0.25 * gamepad2.right_trigger);
                robot.linearSlidesMotor2.setPower(-0.25 * gamepad2.right_trigger);
            }
        } else if (gamepad2.left_trigger > 0.1) {
            telemetry.addData("LS Direction", "UP");
            robot.linearSlidesMotor1.setPower(0.5 * gamepad2.left_trigger);
            robot.linearSlidesMotor2.setPower(0.5 * gamepad2.left_trigger);
        } else {
            telemetry.addData("LS Direction", "OFF");
            robot.linearSlidesMotor1.setPower(0);
            robot.linearSlidesMotor2.setPower(0);
        }


        telemetry.addLine("Test Controls and Positions (GP1):");
        telemetry.addData("Airplane Servo (X=inc, B=dec)", robot.airplaneServo.getPosition());
        telemetry.addData("Wrist Servo (Y=inc, A=dec)", robot.wristServo.getPosition());
        telemetry.addData("Elbow Servo (DPup=inc, DPdn=dec)", robot.elbowServo.getPosition());
        telemetry.addData("Claw Servo (DPleft=inc, DPright=dec)", robot.clawServo.getPosition());
        telemetry.addData("Garage Door Servo (LB=inc, RB=dec)", robot.garageDoorServo.getPosition());
        telemetry.addData("LS Motor 1 Position", robot.linearSlidesMotor1.getCurrentPosition());
        telemetry.addData("LS Motor 2 Position", robot.linearSlidesMotor2.getCurrentPosition());
        telemetry.update();
    }
}
