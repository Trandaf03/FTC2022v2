package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwareMapIDs;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;
import org.firstinspires.ftc.teamcode.util.gyroUtil;

import java.util.List;

public class Drive {

    private HardwareMap hardwareMap;
    private hardwareMapIDs id = new hardwareMapIDs();

    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    private double globalAngle = 0;
    private double grade;
    gyroUtil gyro = new gyroUtil();



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
        gyro.gyroInit(hardwareMap);



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

    /*
    GYRO
     */
    public void resetAngle() {
        grade = gyro.returnAngle(gyroUtil.ROBOT_GYRO_DIRECTION.HEADING);
        globalAngle = 0;
    }
    public double getAngle() {

        double angles = gyro.returnAngle(gyroUtil.ROBOT_GYRO_DIRECTION.HEADING);

        double rotationAngle = angles - grade;
        if (rotationAngle < -180)
            rotationAngle += 360;
        else if (rotationAngle > 180)
            rotationAngle -= 360;
        globalAngle += rotationAngle;
        grade = angles;
        return globalAngle;
    }
    public double checkDirection() {
        double corectie, unghi, unghi_corectie = .10;

        unghi = getAngle();

        if (unghi == 0)
            corectie = 0;
        else
            corectie = -unghi;

        corectie = corectie * unghi_corectie;
        return corectie;
    }

    public void spin(double degrees, double power) throws InterruptedException {

        double  lp, rp;
        resetAngle();

        if (degrees < 0)
        {   // left rotation
            lp = -power;
            rp = power;
        }
        else if (degrees > 0)
        {   // right rotation
            lp = power;
            rp = -power;
        }
        else return;

        leftFront.setPower(lp);
        leftRear.setPower(lp);
        rightRear.setPower(rp);
        rightFront.setPower(rp);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        stop();
        resetAngle();
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
