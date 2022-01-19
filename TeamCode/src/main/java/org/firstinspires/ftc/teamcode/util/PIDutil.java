package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;
import java.util.List;

public class PIDutil {

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    List<DcMotorEx> dcMotorExList;

    private static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    private PIDCoefficients v1pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v2pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v3pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v4pidGains = new PIDCoefficients(0,0,0);
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    private double v1IntegralPower = 0;
    private double v2IntegralPower = 0;
    private double v3IntegralPower = 0;
    private double v4IntegralPower = 0;

    private double v1LastError = 0;
    private double v2LastError = 0;
    private double v3LastError = 0;
    private double v4LastError = 0;

    public double leftFrontPower;
    public double rightFrontPower;
    public double leftRearPower;
    public double rightRearPower;

    public PIDutil(List<DcMotorEx> dcMotorExList){
       this.dcMotorExList = dcMotorExList;

       this.leftFront = dcMotorExList.get(1);
       this.leftRear = dcMotorExList.get(2);
       this.rightFront = dcMotorExList.get(3);
       this.rightRear = dcMotorExList.get(4);

    }
    List<Double> motorPower;

    public List<Double> PIDCalculation(double velocity){
        pidTimer.reset();
        final double v1TargetVelocity = velocity;
        final double v2TargetVelocity = velocity;
        final double v3TargetVelocity = velocity;
        final double v4TargetVelocity = velocity;

        double v1CurrentVelocity = leftFront.getVelocity();
        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v1Error = v1TargetVelocity - v1CurrentVelocity;
        double v2Error = v2TargetVelocity - v2CurrentVelocity;
        double v3Error = v3TargetVelocity - v3CurrentVelocity;
        double v4Error = v4TargetVelocity - v4CurrentVelocity;

        v1IntegralPower += v1Error * pidTimer.time();
        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        double v1deltaError = v1Error - v1LastError;
        double v2deltaError = v1Error - v2LastError;
        double v3deltaError = v1Error - v3LastError;
        double v4deltaError = v1Error - v4LastError;

        double v1Derivative = v1deltaError / pidTimer.time();
        double v2Derivative = v2deltaError / pidTimer.time();
        double v3Derivative = v3deltaError / pidTimer.time();
        double v4Derivative = v4deltaError / pidTimer.time();

        v1pidGains.p = pidCoefficients.p * v1Error;
        v2pidGains.p = pidCoefficients.p * v2Error;
        v3pidGains.p = pidCoefficients.p * v3Error;
        v4pidGains.p = pidCoefficients.p * v4Error;

        v1pidGains.i = pidCoefficients.i * v1IntegralPower;
        v2pidGains.i = pidCoefficients.i * v2IntegralPower;
        v3pidGains.i = pidCoefficients.i * v3IntegralPower;
        v4pidGains.i = pidCoefficients.i * v4IntegralPower;

        v1pidGains.d = pidCoefficients.d * v1Derivative;
        v2pidGains.d = pidCoefficients.d * v2Derivative;
        v3pidGains.d = pidCoefficients.d * v3Derivative;
        v4pidGains.d = pidCoefficients.d * v4Derivative;

        leftFrontPower = v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity;
        rightFrontPower = v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity;
        leftRearPower = v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity;
        rightRearPower = v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity;

        v1LastError = v1Error;
        v2LastError = v2Error;
        v3LastError = v3Error;
        v4LastError = v4Error;

        motorPower = new ArrayList<>();

        motorPower.add(leftFrontPower);
        motorPower.add(rightFrontPower);
        motorPower.add(leftRearPower);
        motorPower.add(rightRearPower);
        return  motorPower;
    }
}
