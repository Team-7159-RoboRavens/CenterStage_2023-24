package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BasicRobots.MecanumDrive;
import org.firstinspires.ftc.teamcode.BasicRobots.TankDrive;

@Config
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;
    public static double ANGLE_DEGREES = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


            int mode = 1;
            telemetry.addLine("Select a mode on gamepad 1:");
            telemetry.addLine("A - Straight forward and back "+DISTANCE+"in. (normal)");
            telemetry.addLine("B - Strafe left and right "+DISTANCE+"in. (for lateral)");
            telemetry.addLine("X - Rotate left and right "+ANGLE_DEGREES+" deg (for angular)");
            telemetry.update();
            while(opModeInInit() || opModeIsActive()){
                if(gamepad1.a){
                    mode = 1;
                    telemetry.addLine("Selected mode A");
                    break;
                } else if (gamepad1.b) {
                    mode = 2;
                    telemetry.addLine("Selected mode B");
                    break;
                } else if (gamepad1.x) {
                    mode = 3;
                    telemetry.addLine("Selected mode X");
                    break;
                }
            }
            if(!isStarted()) telemetry.addLine("Waiting for OpMode to start...");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                if(mode == 1){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(DISTANCE, 0))
                                    .strafeTo(new Vector2d(0, 0))
                                    .build());
                }else if(mode == 2){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(0, DISTANCE))
                                    .strafeTo(new Vector2d(0,0))
                                    .build());
                }else if(mode == 3){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turnTo(Math.toRadians(ANGLE_DEGREES))
                                    .turnTo(0)
                                    .build());
                }

            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else {
            throw new AssertionError();
        }
    }
}
