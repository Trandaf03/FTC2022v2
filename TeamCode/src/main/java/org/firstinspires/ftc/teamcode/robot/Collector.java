package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;

public class Collector {

    private DcMotorEx collectorMotor = null;
    private hardwareMapIDs id = new hardwareMapIDs();

    public Collector(HardwareMap hardwareMap){
        collectorMotor = hardwareMap.get(DcMotorEx.class, id.collector);
    }


    public enum COLLECTING_DIRECTION{
        FORWARD, REVERSE;
    }

    public void startCollector(double power,COLLECTING_DIRECTION direction){
        if(direction == COLLECTING_DIRECTION.FORWARD){
            collectorMotor.setPower(Math.abs(power));
        }
        if(direction == COLLECTING_DIRECTION.REVERSE){
            collectorMotor.setPower(-Math.abs(power));
        }
    }

    public void stop(){
        collectorMotor.setPower(0);
    }

}
