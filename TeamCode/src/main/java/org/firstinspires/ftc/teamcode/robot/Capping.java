package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;

public class Capping {

    public Servo servoCap = null;
    hardwareMapIDs id = new hardwareMapIDs();
    HardwareMap hardwareMap;

    public enum capping_pos{
        down, up;
    }

    public Capping(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        servoCap = this.hardwareMap.get(Servo.class, id.teammarker);
    }

    public void capping(capping_pos pos){
        switch (pos){
            case up:
                servoCap.setPosition(0.15);
                break;
            case down:
                servoCap.setPosition(0);
                break;
            default:
                break;
        }
    }



}
