package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardwareMapIDs;
import org.firstinspires.ftc.teamcode.robot.Drive;

import java.util.Locale;

public class gyroUtil {

    private BNO055IMU imuSensor;
    private Orientation robotOrientation;



    private hardwareMapIDs ids;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private hardwareMapIDs id = new hardwareMapIDs();


    private double globalAngle = 0;
    private double grade;

    public void gyroInit(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.useExternalCrystal   = true;

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor = hardwareMap.get(BNO055IMU.class, "imu");

        imuSensor.initialize(parameters);
    }

    public enum ROBOT_GYRO_DIRECTION {
        HEADING, ROLL, PITCH;
    }

    public gyroUtil() {

    }

    public double returnAngle(ROBOT_GYRO_DIRECTION robotDirection) {
        robotOrientation = imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        switch (robotDirection) {
            case HEADING:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.firstAngle));
            case ROLL:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.secondAngle));
            case PITCH:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.thirdAngle));
            default:
                return 0;
        }
    }


    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }




}
