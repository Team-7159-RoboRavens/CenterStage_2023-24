package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double PAR0_Y_TICKS = -8690.836155316303; //LEFT
        public double PAR1_Y_TICKS = 8375.214382937604; //RIGHT
        public double PERP_X_TICKS = -8142.0059177830035;
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        //TODO: Names, Direction(?)
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));

        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.PAR0_Y_TICKS * par1PosDelta - PARAMS.PAR1_Y_TICKS * par0PosDelta) / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS),
                                (PARAMS.PAR0_Y_TICKS * par1PosVel.velocity - PARAMS.PAR1_Y_TICKS * par0PosVel.velocity) / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.PERP_X_TICKS / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.PERP_X_TICKS / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.PAR0_Y_TICKS - PARAMS.PAR1_Y_TICKS),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
