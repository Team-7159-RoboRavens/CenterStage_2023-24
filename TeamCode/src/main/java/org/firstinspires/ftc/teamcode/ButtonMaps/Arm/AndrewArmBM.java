package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Config
public class AndrewArmBM extends AbstractButtonMap {
    public static double slideUpPower = 0.5;
    public static double slideDownPower = 0.35;
    public static double holdPower = 0.04;

    //TODO: Magic Numbers!!!
    private ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        /*
            Button Map
            Y - Extend the Linear Slides (DONE)
            A - Detract the Linear Slides (DONE)
            B - Release Pixel (Maybe close as well)
            D-Pad - Release airplane (single press) (DONE)

         */

        /* Linear Slides */
        if (opMode.gamepad2.a) {
            if (robot.linearSlidesMotor1.getCurrentPosition() < -5 || robot.linearSlidesMotor2.getCurrentPosition() < -5) {
                opMode.telemetry.addData("LS Direction", "INHIBIT DOWN");
                robot.linearSlidesMotor1.setPower(0);
                robot.linearSlidesMotor2.setPower(0);
            } else {
                opMode.telemetry.addData("LS Direction", "DOWN");
                robot.linearSlidesMotor1.setPower(-slideDownPower);
                robot.linearSlidesMotor2.setPower(-slideDownPower);
            }
        } else if (opMode.gamepad2.y) {
            opMode.telemetry.addData("LS Direction", "UP");
            robot.linearSlidesMotor1.setPower(slideUpPower);
            robot.linearSlidesMotor2.setPower(slideUpPower);
        } else {
            opMode.telemetry.addData("LS Direction", "OFF+HOLD");
            //Small amount of power for hold mode
            robot.linearSlidesMotor1.setPower(holdPower);
            robot.linearSlidesMotor2.setPower(holdPower);
        }


        //Plane Servo (dpad)
        //TODO: find position
        if(opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
            robot.airplaneServo.setPosition(0);
        }
    }
}
