package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Config
public class KrishArmBM extends AbstractButtonMap {
    public static double linearSlidesDownMultiplier = 0.35;
    public static double linearSlidesUpMultiplier = 0.5;
    public static double holdModePower = 0.04;

    private boolean holdMode = false;
    private boolean outputServo = false;

    private ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lsToggleTime = 0;
    private double servoToggleTime = 0;

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        //Linear Slides (on triggers)
        //Copy+Paste from last year lol
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
            if (holdMode) {
                //Small amount of power for hold mode
                robot.linearSlidesMotor1.setPower(holdModePower);
                robot.linearSlidesMotor2.setPower(holdModePower);
            } else {
                robot.linearSlidesMotor1.setPower(0);
                robot.linearSlidesMotor2.setPower(0);
            }
        }
        //Linear Slide Hold Mode
        if (opMode.gamepad2.left_bumper && et.time()-lsToggleTime > 500) {
            holdMode = !holdMode;
            lsToggleTime = et.time();
        }
        opMode.telemetry.addData("LS Hold Mode", holdMode);

        //Output Servo
        if(opMode.gamepad2.x && et.time()-servoToggleTime > 300){
            if(outputServo) robot.clawServo.setPosition(1);
            else robot.clawServo.setPosition(0);
            outputServo = !outputServo;
            servoToggleTime = et.time();
        }

        //Plane Servo
        //TODO: find position
        if(opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
            robot.airplaneServo.setPosition(0);
        }
    }
}
