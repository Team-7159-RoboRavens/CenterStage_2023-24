package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;
@Config
public class KrishDriveBM extends AbstractButtonMap{
    //TODO: Change back to final when done with dash
    public static double fastStrafePower = 0.8;
    public static double mediumStrafePower = 0.5;
    public static double slowStrafePower = 0.2;

    private boolean buttonPressed = false;
    private boolean motorBrake = true;
    private ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double currentStrafeMotorPower;
    private MotorPowers mp;


    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        //Set mp to zero at the start, if no buttons are pressed it will never change
        mp = new MotorPowers(0);

        //Reset Yaw for FOD
        if(opMode.gamepad1.back){
            robot.imu.resetYaw();
        }

        //Motor Brake (toggle) w/ cooldown to avoid rapidly flipping
        if(opMode.gamepad1.b && et.time() > 500){
            et.reset();
            if(motorBrake){
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBrake = false;
            }else{
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBrake = true;
            }
        }
        //Displaying current motor brake status
        if(motorBrake){
            opMode.telemetry.addData("Drive Motor Mode", "Brake");
        }else{
            opMode.telemetry.addData("Drive Motor Mode", "Coast");
        }

        //Speed Settings (hold)
        if(opMode.gamepad1.left_bumper){
            currentStrafeMotorPower = slowStrafePower;
        }else if(opMode.gamepad1.right_bumper){
            currentStrafeMotorPower = fastStrafePower;
        }else{
            currentStrafeMotorPower = mediumStrafePower;
        }

        //DPads OctoStrafe and Trigger Arc Turn, 2nd priority
        MotorPowers dpadMotorPowers = DPadControl.dpadStrafe(opMode.gamepad1, currentStrafeMotorPower);
        MotorPowers triggerMotorPowers = new MotorPowers(0);
        if(opMode.gamepad1.right_trigger > 0.1){
            //Right
            triggerMotorPowers = new MotorPowers(0, opMode.gamepad1.right_trigger*currentStrafeMotorPower, 0, opMode.gamepad1.right_trigger*currentStrafeMotorPower);
        }else if(opMode.gamepad1.left_trigger > 0.1){
            //Left
            triggerMotorPowers = new MotorPowers(opMode.gamepad1.left_trigger*currentStrafeMotorPower, 0, opMode.gamepad1.left_trigger*currentStrafeMotorPower, 0);
        }
        if(dpadMotorPowers.isNotZero() || triggerMotorPowers.isNotZero()){
            mp = dpadMotorPowers;
            mp.combineWith(triggerMotorPowers);
        }

        //Field Oriented Drive (joysticks), 1st priority
        MotorPowers fodMotorPowers = FieldOrientedDrive.fieldOrientedDrive(opMode.gamepad1, robot.imu, currentStrafeMotorPower);
        if(fodMotorPowers.isNotZero()){
            mp = fodMotorPowers;
        }

        //Actually apply the power
        robot.setMotorPowers(mp);
    }
}
