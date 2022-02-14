package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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



        odometryUtil odometry = new odometryUtil(hardwareMap,drive,telemetry);

        slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);
        gyro.gyroInit(hardwareMap);


        waitForStart();
        if (opModeIsActive() && !isStopRequested()){


            this.sleep(1000);

            drive.spin(90, 0.25);


            this.sleep(2000);




        }

    }
}
