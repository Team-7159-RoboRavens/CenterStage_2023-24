package org.firstinspires.ftc.teamcode.ComplexRobots;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BasicRobots.MecanumDrive;

@Config
public class CenterStageRobot extends MecanumDrive {
    //Motors
    public final DcMotorEx linearSlidesMotor1;
    public final DcMotorEx linearSlidesMotor2;

    //Servos
    public final Servo clawServo;
    public final Servo wristServo;
    public final Servo elbowServo;

    public final Servo garageDoorServo;
    public final Servo airplaneServo;
    public final Servo purplePixelServo;

    //Constants
    public static int slidesRaisePosition = 450;
    public static double elbowBackboardPosition = 0.2;
    public static double elbowRaisePosition = 0.9;
    public static double elbowLoadPosition = 0.96;
    public static double wristBackboardPosition = 0.76;
    public static double wristCollapsePosition = 0.9;
    public static double wristLoadPosition = 0;

    //Camera Positions
    public static int leftPlacementLowerBound = 35;
    public static int leftPlacementUpperBound = 65;
    public static int centerPlacementLowerBound = 320;
    public static int centerPlacementUpperBound = 350;
    public static int rightPlacementLowerBound = 500;
    public static int rightPlacementUpperBound = 520;

    //Constructor
    public CenterStageRobot(HardwareMap hardwareMap, Pose2d pose, OpMode opMode) {
        super(hardwareMap, pose, opMode);

        //Linear Slide Motors
        linearSlidesMotor1 = hardwareMap.get(DcMotorEx.class, "linearSlidesMotor1");
        linearSlidesMotor2 = hardwareMap.get(DcMotorEx.class, "linearSlidesMotor2");
        //Setup
        linearSlidesMotor1.setDirection(DcMotor.Direction.REVERSE);
        linearSlidesMotor2.setDirection(DcMotor.Direction.FORWARD);
        linearSlidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlidesMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlidesMotor1.setTargetPositionTolerance(15);
        linearSlidesMotor2.setTargetPositionTolerance(15);
        linearSlidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidesMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlidesMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Output Servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.scaleRange(0,0.35);
        //Force to be in the right place
        clawServo.setPosition(0);

        //Initialize Wrist Servo
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        wristServo.scaleRange(0.14,0.62);
        //Force to be in the right place
        wristServo.setPosition(1);

        //Initialize Elbow Servo
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        //TODO: find numbers
        elbowServo.scaleRange(0,1);
        //Force to be in the right place
        //TODO: find number
        elbowServo.setPosition(1);

        //Initialize Garage Door Servo
        garageDoorServo = hardwareMap.get(Servo.class, "garageDoorServo");
        //TODO: find numbers
        garageDoorServo.scaleRange(0.19,0.72);
        //Force to be in the right place
        //TODO: find number
        garageDoorServo.setPosition(0);

        //Initialize Airplane Servo
        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");
        //TODO: find numbers
        airplaneServo.scaleRange(0.85,1);
        //Force to be in the right place
        //TODO: find number
        airplaneServo.setPosition(1);

        //Initialize Airplane Servo
        purplePixelServo = hardwareMap.get(Servo.class, "purplePixelServo");
        //TODO: find numbers
        purplePixelServo.scaleRange(0.5,1);
        //Force to be in the right place
        //TODO: find number
        purplePixelServo.setPosition(1);
    }

    //TODO: Linear slide helper methods for auto (later)
    public Action setSlideHeightAction(int targetPosition){
        return new SlideHeight(targetPosition);
    }

    class SlideHeight implements Action {
        private boolean initialized;
        private int targetPosition;
        SlideHeight(int targetPosition){
            initialized = false;
            this.targetPosition = targetPosition;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                linearSlidesMotor1.setTargetPosition(this.targetPosition);
                linearSlidesMotor2.setTargetPosition(this.targetPosition);
                linearSlidesMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlidesMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlidesMotor1.setPower(0.6);
                linearSlidesMotor1.setPower(0.6);
                initialized = true;
                return true;
            }
            return linearSlidesMotor1.isBusy() || linearSlidesMotor2.isBusy();
        }
    }
}
