package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareMapIDs;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Slider;

public class odometryUtil {
    private DcMotorEx forwardEncoder = null;
    private DcMotorEx leftEncoder = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Drive drive;
    private hardwareMapIDs IDs = new hardwareMapIDs();

    Slider slider;
    boolean running = false;

    private static final double COUNTS_PER_MOTOR_REV = 8949.99995556 / 6;
    private static final double DRIVE_GEAR_REDUCTION = 1;
    private static final double WHEEL_DIAMETER_CM = 4;
    public static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);



    public double returnY() {
        return Math.abs(forwardEncoder.getCurrentPosition());
    }

    public double returnX() { return Math.abs(leftEncoder.getCurrentPosition()); }

    public odometryUtil(HardwareMap hardwareMap, Drive drive, Telemetry telemetry) {

        this.drive = drive;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        Slider slider  = new Slider(hardwareMap);
        this.slider = slider;

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
    public void driveY_and_lift(double distance, double power, int directie) {

        drive.enableMotors();
        running = true;

        distance = Math.abs(distance * COUNTS_PER_CM);

        resetForword();

        drive.straightPower(power);

        while (running  ) {
            slider.startCulisanta(0.75*directie);
            slider.stopCulisanta(Slider.sliderPos.HIGH_POS);

            if(returnY() > Math.abs(distance) ){
                drive.stop();
            }

            telemetry.addData("acum sunt la cm ", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }
        boolean running = false;

        drive.disableMotors();
    }

    public void driveY(double distance, double power) {

        drive.enableMotors();
        distance = Math.abs(distance * COUNTS_PER_CM);

        resetForword();

        drive.straightPower(power);

        while (returnY() < Math.abs(distance)  ) {

            telemetry.addData("acum sunt la cm ", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }

    //stanga dreapta
    public void driveX_and_lift(double distance, double power) {
        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;

        resetLeft();

        drive.strafePower(power);
        while (returnX() < Math.abs(distance)) {
            slider.startCulisanta(0.75);
            slider.stopCulisanta(Slider.sliderPos.HIGH_POS);
            telemetry.addData("acum sunt la cm ", leftEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();
        }

        drive.stop();
        drive.disableMotors();
    }

    public void driveX(double distance, double power) {
        drive.enableMotors();
        distance = distance * COUNTS_PER_CM;

        resetLeft();

        drive.strafePower(power);
        while (Math.abs(leftEncoder.getCurrentPosition()) < Math.abs(distance)) {
            telemetry.addData("acum sunt la cm", leftEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.update();

        }

        drive.stop();
        drive.disableMotors();
    }


    private ElapsedTime time = new ElapsedTime();



    public void diagonala(double xDistance, double yDistance, double speed, double t, int directie) throws InterruptedException {

        resetAll();

        double distance = Math.hypot(xDistance, yDistance) * COUNTS_PER_CM;

        double encoder_distance = Math.hypot(leftEncoder.getCurrentPosition(), forwardEncoder.getCurrentPosition());


        drive.enableMotors();

        double y = -yDistance;
        double x = xDistance;
        double rx = t;


        double r = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        do {
            double v1 = ((y + x + rx) / r )* speed * directie;
            double v2 = ((y - x + rx) / r)*speed*directie;
            double v3 = ((y - x - rx) / r)*speed*directie;
            double v4 = ((y + x - rx) / r)*speed*directie;
            drive.setCustomPower(v1, v2, v3, v4);

            telemetry.addData("y ", forwardEncoder.getCurrentPosition() / COUNTS_PER_CM);
            telemetry.addData("x ", leftEncoder.getCurrentPosition() / COUNTS_PER_CM);

            telemetry.update();
        } while (returnY()/ COUNTS_PER_CM < yDistance && returnX()/ COUNTS_PER_CM < xDistance);

        drive.stop();
        drive.enableMotors();

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
