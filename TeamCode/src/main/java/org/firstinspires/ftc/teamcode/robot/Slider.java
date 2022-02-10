package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;

public class Slider {
    public DcMotorEx sliderMotor = null;
    public Servo rotationServo = null;
    private hardwareMapIDs id = new hardwareMapIDs();
    private double sliderSpeed = 0.25;
    private boolean culisantaRunnig = false;

    public enum servoPos {
        SERVO_DOWN_POS, SERVO_UP_POS,SERVO_COLECTAT_POS;
    }

    public enum sliderPos {
        LOW_POS, MID_POS, HIGH_POS, ZERO_POS
    }

    private sliderPos lastSliderPos = sliderPos.ZERO_POS;

    public Slider(HardwareMap hardwareMap) {
        sliderMotor = hardwareMap.get(DcMotorEx.class, id.sliderMotor);
        rotationServo = hardwareMap.get(Servo.class, id.sliderServo);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double returnPos(){return Math.abs(sliderMotor.getCurrentPosition()); }

    public void liftSlider(sliderPos position, double speed){

        culisantaRunnig = true;
        sliderMotor.setPower(speed);
        while(culisantaRunnig){
            if(returnPos() > returnPositionTicks(position)){
                sliderMotor.setPower(0);
                culisantaRunnig = false;
            }
        }
        sliderMotor.setPower(0);

    }

    public void startCulisanta( double speed){
        sliderMotor.setPower(speed);
    }
     public void stopCulisanta(sliderPos position){
         if(returnPos() > returnPositionTicks(position)){
             sliderMotor.setPower(0);
         }

     }

    public DcMotorEx returnSlider() {
        return sliderMotor;
    }

    public void setServoPosition(servoPos position) {
        switch (position) {
            case SERVO_COLECTAT_POS:
                rotationServo.setPosition(0.3);
                break;
            case SERVO_DOWN_POS:
                rotationServo.setPosition(0.43);
                break;

            case SERVO_UP_POS:
                rotationServo.setPosition(0.25);
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
                return 0;
            default:
                break;
        }
        return 0;
    }
    public int lastPostion = 0;
    public double errorTics = 10;

    public void verificare(){
        double currentPosition = sliderMotor.getCurrentPosition();
        if(currentPosition == lastPostion - errorTics || currentPosition == lastPostion + errorTics){
            sliderMotor.setPower(0);
        }
    }


}
