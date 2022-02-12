package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class encoderUtil {

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    public encoderUtil(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
    }

    public enum encoderMode {
        STOP_AND_RESET, RUN_USING, RUN_WITHOUT, TO_POSITION
    }

    public encoderUtil() {

    }

    @NonNull
    public  void setEncoderMode(encoderMode encoderMode) {
        switch (encoderMode) {
            case STOP_AND_RESET:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case RUN_USING:
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT:
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TO_POSITION:
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            default:
                break;

        }
    }

}
