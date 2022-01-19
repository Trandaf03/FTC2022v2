package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareMapIDs;
import org.firstinspires.ftc.teamcode.robot.Drive;

public class odometryUtil {
    private DcMotorEx forwardEncoder = null;
    private DcMotorEx leftEncoder = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Drive drive;
    private hardwareMapIDs IDs = new hardwareMapIDs();


    private static final double COUNTS_PER_MOTOR_REV = 8949.99995556 / 6;
    private static final double DRIVE_GEAR_REDUCTION = 1;
    private static final double WHEEL_DIAMETER_CM = 4;
    public static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    public double returnY() {
        return forwardEncoder.getCurrentPosition();
    }

    public double returnX() {
        return leftEncoder.getCurrentPosition();
    }

    public odometryUtil(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        forwardEncoder = hardwareMap.get(DcMotorEx.class, IDs.xOdometry);
        leftEncoder = hardwareMap.get(DcMotorEx.class, IDs.yOdometry);

        forwardEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //fata spate
    public void driveX(double distance, double power) {

        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.straightPower(power);

        while (Math.abs(forwardEncoder.getCurrentPosition()) < Math.abs(distance)) {
            telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }


    //stanga dreapta
    public void driveY(double distance, double power) {
        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;


        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.strafePower(power);
        while (forwardEncoder.getCurrentPosition() < distance) {
            telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }


}
