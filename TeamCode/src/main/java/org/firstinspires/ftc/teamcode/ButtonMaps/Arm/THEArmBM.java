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
    private boolean garageDoorOpen = true;
    //0 = load, 1 = raise, 2 = backboard
    private int armPosition = 0;

    private ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double clawServoToggleTime = 0;
    private double garageServoToggleTime = 0;
    private double elbowServoMoveTime = 0;
    private double wristServoMoveTime = 0;


    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        /* BUTTON MAP
         * A - Elbow/Wrist Intake
         * B - Elbow/Wrist Lift
         * Y - Elbow/Wrist Backboard
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
        if(opMode.gamepad2.right_bumper && et.time()-clawServoToggleTime > 500){
            if(clawOpen) robot.clawServo.setPosition(0);
            else robot.clawServo.setPosition(1);
            clawOpen = !clawOpen;
            clawServoToggleTime = et.time();
        }

        //LB - Garage Door Toggle
        //TODO: find positions
        if(opMode.gamepad2.left_bumper && et.time()-garageServoToggleTime > 500){
            if(garageDoorOpen) robot.garageDoorServo.setPosition(0);
            else robot.garageDoorServo.setPosition(1);
            garageDoorOpen = !garageDoorOpen;
            garageServoToggleTime = et.time();
        }

        if(!clawOpen) {
            if (opMode.gamepad2.y) {
                //Y - Elbow and Wrist to Backboard
                robot.elbowServo.setPosition(CenterStageRobot.elbowBackboardPosition);
                robot.wristServo.setPosition(CenterStageRobot.wristBackboardPosition);
            } else if (opMode.gamepad2.b) {
                //B - Elbow to slightly lifted position
                robot.elbowServo.setPosition(CenterStageRobot.elbowRaisePosition);
                robot.wristServo.setPosition(CenterStageRobot.wristCollapsePosition);
            } else if (opMode.gamepad2.a) {
                //A - Elbow and wrist to Load position
                robot.elbowServo.setPosition(CenterStageRobot.elbowLoadPosition);
                robot.wristServo.setPosition(CenterStageRobot.wristLoadPosition);
            }
        }

        //DPad - Plane Servo
        if(opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
            robot.airplaneServo.setPosition(0);
        }
    }
}
