package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class directionUtil {

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    public directionUtil(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
    }

    public enum direction {
        FORWARD, REVERSE, LEFT, RIGHT
    }

    public directionUtil() {

    }

    @NonNull
    public void setRobotDirection(direction direction) {
        switch (direction) {
            case FORWARD:
                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case REVERSE:
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case LEFT:
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case RIGHT:
                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            default:
                break;
        }
    }

}
