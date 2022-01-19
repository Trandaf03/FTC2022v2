package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;

public class Slider {
    private DcMotorEx sliderMotor = null;
    private Servo rotationServo = null;
    private hardwareMapIDs id = new hardwareMapIDs();
    private double sliderSpeed = 0.25;

    public enum servoPos {
        SERVO_DOWN_POS, SERVO_UP_POS, SERVO_MID_POS;
    }

    public enum sliderPos {
        LOW_POS, MID_POS, HIGH_POS, ZERO_POS
    }

    private sliderPos lastSliderPos = sliderPos.ZERO_POS;

    public Slider(HardwareMap hardwareMap) {
        sliderMotor = hardwareMap.get(DcMotorEx.class, id.sliderMotor);
        rotationServo = hardwareMap.get(Servo.class, id.sliderServo);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void sliderGoToPosition(sliderPos position) {
        sliderMotor.setMotorEnable();
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setTargetPosition(returnPositionTicks(position));
        sliderMotor.setPower(sliderSpeed);
        while (sliderMotor.isBusy());
        sliderMotor.setPower(0);
        sliderMotor.setMotorDisable();
        lastSliderPos = position;
    }

    public DcMotorEx returnSlider() {
        return sliderMotor;
    }

    public void setServoPosition(servoPos position) {
        switch (position) {
            case SERVO_DOWN_POS:
                rotationServo.setPosition(0.05);
                break;
            case SERVO_MID_POS:
                rotationServo.setPosition(0.15);
                break;
            case SERVO_UP_POS:
                rotationServo.setPosition(0.23);
                break;
            default:
                break;
        }
    }

    private int returnPositionTicks(sliderPos position) {
        switch (position) {
            case LOW_POS:
                return 1300;
            case MID_POS:
                return 2300;
            case HIGH_POS:
                return 2900;
            case ZERO_POS:
                return -returnPositionTicks(lastSliderPos);
            default:
                break;
        }
        return 0;
    }


}
