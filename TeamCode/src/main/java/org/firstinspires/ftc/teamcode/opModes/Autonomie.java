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
import org.firstinspires.ftc.teamcode.util.odometryUtil;


@Autonomous (name = "hautonomie")
public class Autonomie extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, breakingModeUtil.breakingMode.BRAKE, directionUtil.direction.FORWARD, encoderUtil.encoderMode.RUN_USING);
        Slider slider = new Slider(hardwareMap);
        Ducky ducky = new Ducky(hardwareMap);


        odometryUtil odometry = new odometryUtil(hardwareMap,drive,telemetry);

        slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);


        waitForStart();
        if (opModeIsActive() && !isStopRequested()){


            odometry.driveY(10, 0.25, isStopRequested());

//            slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);
//
//            odometry.driveY_and_lift(38, 0.25, 1);
//            slider.setServoPosition(Slider.servoPos.SERVO_DOWN_POS);
//            this.sleep(1000);
//
//
//            slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);






            this.sleep(2000);




        }

    }
}
