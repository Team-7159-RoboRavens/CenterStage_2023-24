package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.ComplexRobots.CenterStageRobot;

@Config
public class NoamAndrewDriveBM extends AbstractButtonMap {
    //TODO: Change back to private final when done with dash
    public static double triggerMultipler = 0.9;
    public static double slowStrafeMultiplier = 0.35;
    public static double powerMultiplier = 0.5;

    private boolean buttonPressed = false;
    private boolean combineWithPivotTurn = false;

    private double currentMotorPower;
    private MotorPowers mp = new MotorPowers(0);

    @Override
    public void loop(CenterStageRobot robot, OpMode opMode) {
        currentMotorPower = 1;
        /*
         * Button A - Complete break
         */
        if (opMode.gamepad1.a) {
            mp.setMotorPowers(0);
            return;
        }

        //Dpad strafe using dpad
        if (opMode.gamepad1.b) {
            currentMotorPower *= slowStrafeMultiplier;
        }
        MotorPowers dpadMotorPowers = DPadControl.dpadStrafe(opMode.gamepad1, currentMotorPower);

        //Field-Oriented Driving using left joystick
        MotorPowers fodMotorPowers = FieldOrientedDrive.fieldOrientedDrive(opMode.gamepad1, robot.imu, currentMotorPower);
        if (fodMotorPowers.isNotZero()) {
            mp = fodMotorPowers;
        }

        /*
         * Pivot turn methods
         */
        //Pivot Turn using joystick
        MotorPowers joystickPivotTurnMotorPowers = robot.pivotTurn(currentMotorPower, opMode.gamepad1.right_stick_x > 0.1, opMode.gamepad1.right_stick_x < -0.1);
        //Pivot Turn Using bumpers
        MotorPowers bumperPivotTurnMotorPowers = robot.pivotTurn(currentMotorPower, opMode.gamepad1.right_bumper, opMode.gamepad1.left_bumper);

        /*
         * Normal Drive
         */
        MotorPowers triggerMotorPowers = new MotorPowers(0);
        //Forward
        if (opMode.gamepad1.right_trigger > 0.1) {
            triggerMotorPowers = new MotorPowers(opMode.gamepad1.right_trigger * triggerMultipler);
        }
        //Backward
        else if (opMode.gamepad1.left_trigger > 0.1) {
            //Backward
            triggerMotorPowers = new MotorPowers(-opMode.gamepad1.left_trigger * triggerMultipler);


        }
    }
}
