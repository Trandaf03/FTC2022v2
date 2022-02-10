package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Ducky;
import org.firstinspires.ftc.teamcode.robot.Slider;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;
import org.firstinspires.ftc.teamcode.util.odometryUtil;

@TeleOp(name = "MainTele")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, breakingModeUtil.breakingMode.BRAKE, directionUtil.direction.FORWARD, encoderUtil.encoderMode.RUN_USING);
        Ducky ducky = new Ducky(hardwareMap);
        Slider slider = new Slider(hardwareMap);
        Collector collector = new Collector(hardwareMap);
        odometryUtil odometry = new odometryUtil(hardwareMap,drive,telemetry);

//        odometryUtil odometry = new odometryUtil();

        ElapsedTime cappingTime = new ElapsedTime();

        
        boolean isDucky = false;
        boolean isCollector = false;
        boolean sus = false;
        boolean sliderJos = false;
        boolean colectat = false;
        boolean culisantaSus = false;

        slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);


        waitForStart();
        cappingTime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            drive.robotControl(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);



            if (this.gamepad1.right_bumper) {
               if(isCollector == false){
                   collector.startCollector(1, Collector.COLLECTING_DIRECTION.FORWARD);
                   isCollector = true;
                   this.sleep(200);
               }
               else{
                   collector.startCollector(0, Collector.COLLECTING_DIRECTION.FORWARD);
                   isCollector = false;
                   this.sleep(200);
               }
            }

            if (this.gamepad1.left_bumper) {
                if(isDucky == false){
                    ducky.startDucky(0.25, Ducky.DUCKY_DIRECTION.FORWARD);
                    isDucky = true;
                    this.sleep(200);
                }
                else{
                    ducky.startDucky(0, Ducky.DUCKY_DIRECTION.FORWARD);
                    isDucky = false;
                    this.sleep(200);
                }
            }

            if(this.gamepad1.triangle){
                if(sliderJos == false){
                    slider.setServoPosition(Slider.servoPos.SERVO_DOWN_POS);

                    sliderJos = true;
                    this.sleep(200);
                }
                else{
                    slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);

                    sliderJos = false;
                    this.sleep(200);
                }
            }

            if(this.gamepad1.circle){
                if(colectat == false){
                    slider.setServoPosition(Slider.servoPos.SERVO_COLECTAT_POS);

                    colectat = true;
                    this.sleep(200);
                }
                else if(colectat==true){
                    slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);
                    colectat = false;
                    this.sleep(200);
                }

            }

            if (this.gamepad1.dpad_up && culisantaSus == false) {
                slider.sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slider.liftSlider(Slider.sliderPos.HIGH_POS, 0.75);
                culisantaSus = true;
                this.sleep(200);
            }
            if (this.gamepad1.dpad_down && culisantaSus) {
                slider.sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slider.liftSlider(Slider.sliderPos.HIGH_POS, -0.55);
                culisantaSus = false;
                this.sleep(200);
            }


            if(this.gamepad1.right_trigger > 0){
                slider.sliderMotor.setPower(0.75*this.gamepad1.right_trigger);
            }
            if(this.gamepad1.left_trigger > 0){
                slider.sliderMotor.setPower(-1*0.5*this.gamepad1.left_trigger);
            }
            if(this.gamepad1.right_trigger == 0 && this.gamepad1.left_trigger == 0){
                slider.sliderMotor.setPower(0);

            }


            //capping
            if(this.gamepad1.y && cappingTime.seconds() > 90){
                sus = !sus;
                this.sleep(200);
            }
            telemetry.addLine("culisanta: "+ String.valueOf(slider.sliderMotor.getCurrentPosition()));
            telemetry.addLine("X: "+ odometry.returnX());
            telemetry.addLine("Y: "+ odometry.returnY());

            telemetry.update();

        }



    }
}
