package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl;
import org.firstinspires.ftc.teamcode.ButtonMaps.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Config
public class NoamAndrewDriveBM extends AbstractButtonMap {
    //TODO: Change back to private final when done with dash
    public static double triggerMultipler = 0.85;
    public static double dpadBumperMultiplier = 0.65;
    public static double fodMultiplier = 0.85;
    public static double slowStrafeMultiplier = 0.35;
    public static double basePower = 0.65;

    private boolean buttonPressed = false;
    private boolean combineWithPivotTurn = false;

    private double currentMotorPower;
    private MotorPowers mp;// = new MotorPowers(0);

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        mp = new MotorPowers(0);
        currentMotorPower = basePower;
        /*
         * Button A - Complete break
         */
        if (opMode.gamepad1.a) {
            robot.setAllMotorPowers(0);
            opMode.telemetry.addLine("Break!!");
            return;
        }

        if(opMode.gamepad1.back){
            robot.imu.resetYaw();
        }

        //Slow Strafe Button
        if (opMode.gamepad1.b) {
            currentMotorPower *= slowStrafeMultiplier;
            opMode.telemetry.addLine("Slow Multiplier Active!");
        }

        //Dpad strafe using dpad
        MotorPowers dpadMotorPowers = DPadControl.dpadStrafe(opMode.gamepad1, currentMotorPower);
        if(dpadMotorPowers.isNotZero()){
            mp = dpadMotorPowers;
            opMode.telemetry.addLine("DPad Active!");
        }

        //Field-Oriented Driving using left joystick
        MotorPowers fodMotorPowers = FieldOrientedDrive.fieldOrientedDrive(opMode.gamepad1, robot.imu, opMode.gamepad1.b ? fodMultiplier*slowStrafeMultiplier : fodMultiplier);
        if (fodMotorPowers.isNotZero()) {
            mp = fodMotorPowers;
            opMode.telemetry.addLine("FOD Active!");
        }

        /*
         * Pivot turn methods
         */
        //Pivot Turn using joystick
//        if(Math.abs(opMode.gamepad1.right_stick_x) > 0.1){
//            MotorPowers joystickPivotTurnMotorPowers = robot.pivotTurn(currentMotorPower*(Math.abs(opMode.gamepad1.right_stick_x)), opMode.gamepad1.right_stick_x > 0.1, opMode.gamepad1.right_stick_x < -0.1);
//            mp = joystickPivotTurnMotorPowers;
//        }

        //Pivot Turn Using bumpers
        if(opMode.gamepad1.right_bumper || opMode.gamepad1.left_bumper){
            MotorPowers bumperPivotTurnMotorPowers = robot.pivotTurn(currentMotorPower, opMode.gamepad1.right_bumper, opMode.gamepad1.left_bumper);
            opMode.telemetry.addLine("Bumper Pivot Turn Active!");
            mp = bumperPivotTurnMotorPowers;
        }

        /*
         * Normal Drive
         */
        MotorPowers triggerMotorPowers = new MotorPowers(0);
        //Forward
        if (opMode.gamepad1.right_trigger > 0.1) {
            triggerMotorPowers = new MotorPowers(opMode.gamepad1.right_trigger * triggerMultipler);
            opMode.telemetry.addLine("Trigger Right (forward) active!");
            mp = triggerMotorPowers;
        }
        //Backward
        else if (opMode.gamepad1.left_trigger > 0.1) {
            //Backward
            triggerMotorPowers = new MotorPowers(-opMode.gamepad1.left_trigger * triggerMultipler);
            opMode.telemetry.addLine("Trigger Left (backward) active!");
            mp = triggerMotorPowers;
        }

        robot.setMotorPowers(mp);
    }
}
