package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Ducky;
import org.firstinspires.ftc.teamcode.robot.Slider;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;
import org.firstinspires.ftc.teamcode.util.gyroUtil;
import org.firstinspires.ftc.teamcode.util.odometryUtil;


@Autonomous (name = "hautonomie")
public class Autonomie extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, breakingModeUtil.breakingMode.BRAKE, directionUtil.direction.FORWARD, encoderUtil.encoderMode.RUN_USING);
        Slider slider = new Slider(hardwareMap);
        Ducky ducky = new Ducky(hardwareMap);
        gyroUtil gyro = new gyroUtil();
        Collector collector = new Collector(hardwareMap);




        odometryUtil odometry = new odometryUtil(hardwareMap,drive,telemetry);

        slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);
        gyro.gyroInit(hardwareMap);


        waitForStart();
        if (opModeIsActive() && !isStopRequested()){


            /*
            TODO test hautonomie caz 1
             */

            slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);

            odometry.driveY_and_lift(38, 0.5, 1);

            slider.setServoPosition(Slider.servoPos.SERVO_DOWN_POS);
            this.sleep(1000);
            slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);
            slider.liftSlider(-0.5);

            drive.spin(30, 0.25);
            collector.startCollector(1, Collector.COLLECTING_DIRECTION.FORWARD);
            odometry.driveY(40, 0.5);
            this.sleep(300);
            slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);
            collector.startCollector(0, Collector.COLLECTING_DIRECTION.FORWARD);


            odometry.driveY_and_lift(40, 0.5, -1);
            drive.spin(-30, 0.25);
            this.sleep(200);
            slider.setServoPosition(Slider.servoPos.SERVO_DOWN_POS);
            this.sleep(1000);
            slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);


            drive.spin(50, 0.25);
            odometry.driveY_and_lift(60, -0.5, -1);
            ducky.startDucky(0.25, Ducky.DUCKY_DIRECTION.FORWARD);
            this.sleep(1000);

















        }



    }


}
