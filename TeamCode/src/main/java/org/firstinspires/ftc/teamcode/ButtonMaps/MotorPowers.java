package org.firstinspires.ftc.teamcode.ButtonMaps;

public class MotorPowers {
    public double leftFront, rightFront, leftBack, rightBack;

    public MotorPowers(double leftFront, double rightFront, double leftBack, double rightBack){
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }
    public MotorPowers(double all){
        this.leftFront = all;
        this.rightFront = all;
        this.leftBack = all;
        this.rightBack = all;
    }

    public boolean isNotZero() {
        return this.leftFront != 0 || this.rightFront != 0 || this.leftBack != 0 || this.rightBack != 0;
    }

    public void setMotorPowers(double allPower){
        this.leftFront = allPower;
        this.rightFront = allPower;
        this.leftBack = allPower;
        this.rightBack = allPower;
    }

    public void combineWith(MotorPowers other){
        double leftFrontMax = Math.max(Math.abs(this.leftFront + other.leftFront), 1);
        double rightFrontMax = Math.max(Math.abs(this.rightFront + other.rightFront), 1);
        double leftBackMax = Math.max(Math.abs(this.leftBack + other.rightBack), 1);
        double rightBackMax = Math.max(Math.abs(this.rightBack + other.rightBack), 1);
        double denominator = Math.max(Math.max(Math.max(leftFrontMax, rightFrontMax),leftBackMax), rightBackMax);
        this.leftFront = (other.leftFront + this.leftFront) / denominator;
        this.rightFront = (other.rightFront + this.rightFront) / denominator;
        this.leftBack = (other.leftBack + this.leftBack) / denominator;
        this.rightBack = (other.rightBack + this.rightBack) / denominator;
    }
}

