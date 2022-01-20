package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;

public class Manuta {

    public Servo manuta = null;
    hardwareMapIDs id = new hardwareMapIDs();

    public void initManuta(HardwareMap hwMap) {
        manuta = hwMap.get(Servo.class, id.teammarker);
    }

    public void manutaJos() {
        manuta.setPosition(0);
    }

    public void manutaSus() {
        manuta.setPosition(0.15);
    }

    public double manutaPosition() {
        return manuta.getPosition();
    }
}
