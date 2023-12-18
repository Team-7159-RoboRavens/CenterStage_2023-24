package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl;
import org.firstinspires.ftc.teamcode.ButtonMaps.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
@Config
public class EitanNadavDriveBM extends AbstractButtonMap {
    //TODO: Change back to final when done with dash
    public static double triggerMultipler = 0.9;
    public static double fastStrafePower = 0.65;
    public static double slowStrafePower = 0.35;

    private boolean buttonPressed = false;
    private boolean combineWithPivotTurn = false;

    private double currentStrafeMotorPower;
    private MotorPowers mp;


    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        buttonPressed = false;
        mp = new MotorPowers(0);

        //Reset Yaw for FOD
        if(opMode.gamepad1.back){
            robot.imu.resetYaw();
        }
//        opMode.telemetry.addData("Robot Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        //Brake button (bypasses everything)
        if(opMode.gamepad1.a){
            robot.setMotorPowers(mp);
            //Seriously, nothing else will run.
            return;
        }

        //Slow button
        if(opMode.gamepad1.y){
            currentStrafeMotorPower = slowStrafePower;
        }else{
            currentStrafeMotorPower = fastStrafePower;
        }

        //Field Oriented Drive (joysticks), 4th priority
        MotorPowers fodMotorPowers = FieldOrientedDrive.fieldOrientedDrive(opMode.gamepad1, robot.imu, fastStrafePower);
        if(fodMotorPowers.isNotZero()){
//            buttonPressed = true;
            mp = fodMotorPowers;
        }

        //Pivot turn is combined with triggers, 3rd priority
        MotorPowers pivotTurnMotorPowers = robot.pivotTurn(currentStrafeMotorPower, opMode.gamepad1.right_bumper, opMode.gamepad1.left_bumper);
        MotorPowers triggerMotorPowers = new MotorPowers(0);
        if(opMode.gamepad1.right_trigger > 0.1){
            //Forward
            triggerMotorPowers = new MotorPowers(opMode.gamepad1.right_trigger*triggerMultipler);
        }else if(opMode.gamepad1.left_trigger > 0.1){
            //Backward
            triggerMotorPowers = new MotorPowers(-opMode.gamepad1.left_trigger*triggerMultipler);
        }
        if(triggerMotorPowers.isNotZero() || pivotTurnMotorPowers.isNotZero()){
//            buttonPressed = true;
            triggerMotorPowers.combineWith(pivotTurnMotorPowers);
            mp = triggerMotorPowers;
        }

        //Dedicated diagonal buttons & pivot turn, 2nd priority
        if(opMode.gamepad1.x){
            //Up+Left
//            buttonPressed = true;
            mp = new MotorPowers(0,currentStrafeMotorPower,currentStrafeMotorPower,0);
            mp.combineWith(pivotTurnMotorPowers);
        }else if(opMode.gamepad1.b){
            //Up+Right
//            buttonPressed = true;
            mp = new MotorPowers(currentStrafeMotorPower, 0, 0, currentStrafeMotorPower);
            mp.combineWith(pivotTurnMotorPowers);
        }

        //DPads OctoStrafe and Pivot Turn, 1st priority
        MotorPowers dpadMotorPowers = DPadControl.dpadStrafe(opMode.gamepad1, currentStrafeMotorPower);
        if(dpadMotorPowers.isNotZero()){
//            buttonPressed = true;
            mp = dpadMotorPowers;
            mp.combineWith(pivotTurnMotorPowers);
        }
        
//        if(!buttonPressed){
//            mp = new MotorPowers(0);
//        }
        robot.setMotorPowers(mp);
    }
}
