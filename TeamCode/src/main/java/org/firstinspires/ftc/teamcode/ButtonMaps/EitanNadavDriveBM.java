package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

public class EitanNadavDriveBM extends AbstractButtonMap{

    private final double triggerMultipler = 1;
    private boolean buttonPressed = false;

    private double powLeftFront = 0;
    private double powRightFront = 0;
    private double powLeftBack = 0;
    private double powRightBack = 0;

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        buttonPressed = false;

        MotorPowers dpadMotorPowers = DPadControl.dpadStrafe(opMode.gamepad1, 0.5);

        if(opMode.gamepad1.right_trigger > 0.1){
            powLeftFront = opMode.gamepad1.right_trigger*triggerMultipler;
            powRightFront = opMode.gamepad1.right_trigger*triggerMultipler;
            powLeftBack = opMode.gamepad1.right_trigger*triggerMultipler;
            powRightBack = opMode.gamepad1.right_trigger*triggerMultipler;
            buttonPressed = true;
        }else if(opMode.gamepad1.left_trigger > 0.1){
            powLeftFront = -opMode.gamepad1.right_trigger*triggerMultipler;
            powRightFront = -opMode.gamepad1.right_trigger*triggerMultipler;
            powLeftBack = -opMode.gamepad1.right_trigger*triggerMultipler;
            powRightBack = -opMode.gamepad1.right_trigger*triggerMultipler;
            buttonPressed = true;
        }

        if(!buttonPressed){
            powLeftFront = 0;
            powRightFront = 0;
            powLeftBack = 0;
            powRightBack = 0;
        }
        robot.setMotorPower(powLeftFront, powRightFront, powLeftBack, powRightBack);
    }
}
