package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Config
public class THEArmBM extends AbstractButtonMap {
    public static double linearSlidesDownMultiplier = 0.35;
    public static double linearSlidesUpMultiplier = 0.55;

    private boolean clawOpen = false;
    private boolean garageDoorOpen = false;

    private ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double clawServoToggleTime = 0;
    private double garageServoToggleTime = 0;
    private double elbowServoMoveTime = 0;
    private double wristServoMoveTime = 0;

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        /* BUTTON MAP
         * X - Elbow Toward Intake
         * Y - Elbow Toward Outtake
         * A - Increase Wrist Angle
         * B - Decrease Wrist Angle
         * Dpad - Release Airplane
         * RB - Claw Toggle
         * LB - Garage Door Toggle
         * RT - Slides Raise
         * LT - Slides Lower
         */

        //Triggers - Slides Raise/Lower
        if (opMode.gamepad2.right_trigger > 0.1) {
            if (robot.linearSlidesMotor1.getCurrentPosition() < -5 || robot.linearSlidesMotor2.getCurrentPosition() < -5) {
                opMode.telemetry.addData("LS Direction", "INHIBIT DOWN");
                robot.linearSlidesMotor1.setPower(0);
                robot.linearSlidesMotor2.setPower(0);
            } else {
                opMode.telemetry.addData("LS Direction", "DOWN");
                robot.linearSlidesMotor1.setPower(-linearSlidesDownMultiplier* opMode.gamepad2.right_trigger);
                robot.linearSlidesMotor2.setPower(-linearSlidesDownMultiplier * opMode.gamepad2.right_trigger);
            }
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            opMode.telemetry.addData("LS Direction", "UP");
            robot.linearSlidesMotor1.setPower(linearSlidesUpMultiplier * opMode.gamepad2.left_trigger);
            robot.linearSlidesMotor2.setPower(linearSlidesUpMultiplier * opMode.gamepad2.left_trigger);
        } else {
            opMode.telemetry.addData("LS Direction", "OFF");
            robot.linearSlidesMotor1.setPower(0);
            robot.linearSlidesMotor2.setPower(0);

        }

        //RB - Claw Toggle
        //TODO: find positions
        if(opMode.gamepad2.right_bumper && et.time()-clawServoToggleTime > 300){
            if(clawOpen) robot.clawServo.setPosition(1);
            else robot.clawServo.setPosition(0);
            clawOpen = !clawOpen;
            clawServoToggleTime = et.time();
        }

        //LB - Garage Door Toggle
        //TODO: find positions
        if(opMode.gamepad2.left_bumper && et.time()-garageServoToggleTime > 300){
            if(garageDoorOpen) robot.garageDoorServo.setPosition(1);
            else robot.garageDoorServo.setPosition(0);
            garageDoorOpen = !garageDoorOpen;
            garageServoToggleTime = et.time();
        }


        //TODO: find +/-
        if(opMode.gamepad2.x && et.time()-elbowServoMoveTime > 35){
            //X - Elbow Toward Intake
            robot.elbowServo.setPosition(robot.elbowServo.getPosition() - 0.05);
            elbowServoMoveTime = et.time();
        }else if(opMode.gamepad2.y && et.time()-elbowServoMoveTime > 35){
            //Y - Elbow Toward Backboard
            robot.elbowServo.setPosition(robot.elbowServo.getPosition() + 0.05);
            elbowServoMoveTime = et.time();
        }

        //TODO: find +/-
        if(opMode.gamepad2.b && et.time()-wristServoMoveTime > 35){
            //B - Wrist Angle Decrease
            robot.wristServo.setPosition(robot.wristServo.getPosition() - 0.05);
            wristServoMoveTime = et.time();
        }else if(opMode.gamepad2.a && et.time()-wristServoMoveTime > 35){
            //A - Wrist Angle Increase
            robot.wristServo.setPosition(robot.wristServo.getPosition() + 0.05);
            wristServoMoveTime = et.time();
        }

        //DPad - Plane Servo
        if(opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
            robot.airplaneServo.setPosition(0);
        }
    }
}
