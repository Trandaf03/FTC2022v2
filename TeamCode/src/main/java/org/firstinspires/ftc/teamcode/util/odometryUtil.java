package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);


    public double returnY() {
        return forwardEncoder.getCurrentPosition();
    }

    public double returnX() {
        return leftEncoder.getCurrentPosition();
    }

    public odometryUtil(HardwareMap hardwareMap, Drive drive, Telemetry telemetry) {

        this.drive = drive;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        forwardEncoder = this.hardwareMap.get(DcMotorEx.class, IDs.xOdometry);
        leftEncoder = this.hardwareMap.get(DcMotorEx.class, IDs.yOdometry);

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
        distance = Math.abs(distance * COUNTS_PER_CM);

        resetForword();

        drive.straightPower(power);

        while (Math.abs(forwardEncoder.getCurrentPosition()) < Math.abs(distance)) {

            telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }


    private ElapsedTime time = new ElapsedTime();

    public void driveX2(double distance, double power) {

        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;
        distance = Math.abs(distance);
        resetForword();
        double maximumSeconds = 1.0;
        double ticsToMaximumSeconds = 0;
        time.reset();
        do {
            if (time.seconds() <= maximumSeconds && Math.abs(forwardEncoder.getCurrentPosition()) <= distance / 2) {
                drive.straightPower(time.seconds());
                ticsToMaximumSeconds = Math.abs(forwardEncoder.getCurrentPosition());
            } else {
                if (Math.abs(forwardEncoder.getCurrentPosition()) <= distance - ticsToMaximumSeconds) {
                    drive.straightPower(1);
                    time.reset();
                } else {
                    drive.straightPower(1.0 - time.seconds());
                }
            }
        } while (Math.abs(forwardEncoder.getCurrentPosition()) <= distance);

        //resolveError(distance, 0.25, forwardEncoder);
        drive.stop();
        drive.disableMotors();
    }

    private void resolveError(double targetPosition, double speed, DcMotorEx encoder) {
        if (Math.abs(encoder.getCurrentPosition()) < targetPosition) {
            do {
                drive.straightPower(speed);
            } while (Math.abs(encoder.getCurrentPosition()) < targetPosition + 10);
            resolveError(targetPosition, speed, encoder);
        } else if (Math.abs(encoder.getCurrentPosition()) > targetPosition - 10) {
            do {
                drive.straightPower(-speed);
            } while (Math.abs(encoder.getCurrentPosition()) > targetPosition);
            resolveError(targetPosition, speed, encoder);
        } else return;
    }


    //stanga dreapta
    public void driveY(double distance, double power) {
        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;

        resetLeft();

        drive.strafePower(power);
        while (leftEncoder.getCurrentPosition() < distance) {
            telemetry.addData("acum sunt la cm", leftEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }

    public void holonomicDrive(double xDistance, double yDistance, double speed, double t) throws InterruptedException {
        xDistance *= 1.1;
        yDistance *= 1.5;
        resetAll();
        double distance = Math.hypot(xDistance, yDistance) * COUNTS_PER_CM;
        double encoder_distance = Math.hypot(leftEncoder.getCurrentPosition(), forwardEncoder.getCurrentPosition());


        drive.enableMotors();

        double y = -yDistance;
        double x = xDistance;
        double rx = t;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        do {
            double v1 = (y + x + rx) / denominator;
            double v2 = (y - x + rx) / denominator;
            double v3 = (y - x - rx) / denominator;
            double v4 = (y + x - rx) / denominator;
            drive.setCustomPower(v1, v2, v3, v4);
        } while (drive.isEntireRobotBusy() && encoder_distance < distance);

        drive.stop();
        drive.enableMotors();

    }

    private void resetLeft() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetForword() {
        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetAll() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
