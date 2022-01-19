package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;


public class Ducky {

    private DcMotorEx ducky = null;
    private hardwareMapIDs id = new hardwareMapIDs();

    public enum DUCKY_DIRECTION {
        FORWARD, REVERSE;
    }

    public Ducky(HardwareMap hardwareMap) {
        ducky = hardwareMap.get(DcMotorEx.class, id.duckRotation);
    }

    public void startDucky(double motorPower, DUCKY_DIRECTION direction) {
        if (direction == DUCKY_DIRECTION.FORWARD) {
            ducky.setPower(Math.abs(motorPower));
        }
        if (direction == DUCKY_DIRECTION.REVERSE) {
            ducky.setPower(-Math.abs(motorPower));
        }

    }

}
