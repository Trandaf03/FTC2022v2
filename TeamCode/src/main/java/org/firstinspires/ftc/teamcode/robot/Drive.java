package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;

import java.util.List;

public class Drive {

    private HardwareMap hardwareMap;
    private hardwareMapIDs id = new hardwareMapIDs();

    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;


    List<DcMotorEx> dcMotorExList;


    public Drive(HardwareMap hardwareMap,
                 breakingModeUtil.breakingMode breakingMode,
                 directionUtil.direction direction,
                 encoderUtil.encoderMode encoderMode) {
        this.hardwareMap = hardwareMap;

        leftFront = this.hardwareMap.get(DcMotorEx.class, id.leftFront);
        rightFront = this.hardwareMap.get(DcMotorEx.class, id.rightFront);
        leftRear = this.hardwareMap.get(DcMotorEx.class, id.leftRear);
        rightRear = this.hardwareMap.get(DcMotorEx.class, id.rightRear);


        breakingModeUtil breaking = new breakingModeUtil(leftFront, leftRear, rightFront, rightRear);
        directionUtil dir = new directionUtil(leftFront, leftRear, rightFront, rightRear);
        encoderUtil mode = new encoderUtil(leftFront, leftRear, rightFront, rightRear);

        breaking.setBreakingMode(breakingMode);
        dir.setRobotDirection(direction);
        mode.setEncoderMode(encoderMode);

        stop();
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void enableMotors() {
        leftFront.setMotorEnable();
        rightFront.setMotorEnable();
        leftRear.setMotorEnable();
        rightRear.setMotorEnable();
    }

    public void disableMotors() {
        leftFront.setMotorDisable();
        rightFront.setMotorDisable();
        leftRear.setMotorDisable();
        rightRear.setMotorDisable();
    }

    public List<DcMotorEx> getDcMotorExList() {
        dcMotorExList.add(leftFront);
        dcMotorExList.add(rightFront);
        dcMotorExList.add(leftRear);
        dcMotorExList.add(rightRear);

        return dcMotorExList;
    }

    public void robotControl2(double left_stick_x, double left_stick_y, double right_stick_x) {
        double r = Math.hypot(left_stick_x, -left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;
        final double v1 = (setVelocity(r * Math.cos(robotAngle)) + rightX)*386.3;
        final double v2 = (setVelocity(r * Math.sin(robotAngle)) - rightX)*386.3;
        final double v3 = (setVelocity(r * Math.sin(robotAngle)) + rightX)*386.3;
        final double v4 = (setVelocity(r * Math.cos(robotAngle)) - rightX)*386.3;

        leftFront.setVelocity(v1);
        rightFront.setVelocity(v2);
        leftRear.setVelocity(v3);
        rightRear.setVelocity(v4);
    }

    public void robotControl(double left_stick_x, double left_stick_y, double right_stick_x) {
        double r = Math.hypot(left_stick_x, -left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;
        final double v1 = (setVelocity(r * Math.cos(robotAngle)) + setVelocity(rightX));
        final double v2 = (setVelocity(r * Math.sin(robotAngle)) - setVelocity(rightX));
        final double v3 = (setVelocity(r * Math.sin(robotAngle)) + setVelocity(rightX));
        final double v4 = (setVelocity(r * Math.cos(robotAngle)) - setVelocity(rightX));

        leftFront.setVelocity(v1);
        rightFront.setVelocity(v2);
        leftRear.setVelocity(v3);
        rightRear.setVelocity(v4);
    }

    private double setVelocity(double power) {
        return power * 3000;
    }

    public void straightPower(double speed) {
        leftFront.setVelocity(setVelocity(speed));
        rightFront.setVelocity(setVelocity(speed));
        leftRear.setVelocity(setVelocity(speed));
        rightRear.setVelocity(setVelocity(speed));
    }

    public void strafePower(double speed) {
        leftFront.setVelocity(setVelocity(-speed));
        rightFront.setVelocity(setVelocity(speed));
        leftRear.setVelocity(setVelocity(speed));
        rightRear.setVelocity(setVelocity(-speed));
    }

    public void setCustomPower(double v1, double v2, double v3, double v4) {
        leftFront.setPower(v1);
        leftRear.setPower(v2);
        rightFront.setPower(v3);
        rightRear.setPower(v4);
    }

    public boolean isEntireRobotBusy() {
        if (leftFront.isBusy())
            return true;
        if (rightFront.isBusy())
            return true;
        if (leftRear.isBusy())
            return true;
        if (rightRear.isBusy())
            return true;
        return false;
    }

}
