package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DPadControl {

    public static MotorPowers dpadStrafe(Gamepad gamepad, double motorPower){
        if(gamepad.dpad_up){
            if(gamepad.dpad_left){
                //Diagonal Forward Left
                return new MotorPowers(0,motorPower,motorPower,0);
            }else if(gamepad.dpad_right){
                //Diagonal Forward Right
                return new MotorPowers(motorPower,0,0,motorPower);
            }else{
                //Straight Forward
                return new MotorPowers(motorPower,motorPower,motorPower,motorPower);
            }
        }else if(gamepad.dpad_down){
            if(gamepad.dpad_left){
                //Diagonal Backward Left
                return new MotorPowers(-motorPower,0,0,-motorPower);
            }else if(gamepad.dpad_right){
                //Diagonal Backward Right
                return new MotorPowers(0,-motorPower,-motorPower,0);
            }else{
                //Straight Backward
                return new MotorPowers(-motorPower,-motorPower,-motorPower,-motorPower);
            }
        }else if(gamepad.dpad_left){
            //Straight Left
            return new MotorPowers(-motorPower,motorPower,motorPower,-motorPower);
        }else if(gamepad.dpad_right){
            //Straight Right
            return new MotorPowers(motorPower,-motorPower,-motorPower,motorPower);

        }else {
            return new MotorPowers(0, 0, 0, 0);
        }
    }


}
