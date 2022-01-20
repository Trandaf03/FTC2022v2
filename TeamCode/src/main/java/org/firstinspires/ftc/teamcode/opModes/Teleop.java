package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Ducky;
import org.firstinspires.ftc.teamcode.robot.Manuta;
import org.firstinspires.ftc.teamcode.robot.Slider;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;

@TeleOp(name = "MainTele")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, breakingModeUtil.breakingMode.BRAKE, directionUtil.direction.FORWARD, encoderUtil.encoderMode.RUN_USING);
        Ducky ducky = new Ducky(hardwareMap);
        Slider slider = new Slider(hardwareMap);
        Collector collector = new Collector(hardwareMap);
        Manuta manuta = new Manuta();


        manuta.initManuta(hardwareMap);
        boolean isDucky = false;
        boolean isCollector = false;
        boolean sus = false;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            drive.robotControl(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);


            if (this.gamepad1.left_bumper) {
                isCollector = !isCollector;
                this.sleep(200);
            }
            if (isCollector) {
                collector.startCollector(1, Collector.COLLECTING_DIRECTION.FORWARD);
            }
            if (this.gamepad1.right_bumper) {
                isDucky = !isDucky;
                this.sleep(200);
            }
            if (isDucky) {
                ducky.startDucky(1, Ducky.DUCKY_DIRECTION.FORWARD);
            }

            if (this.gamepad1.dpad_down) {
                slider.sliderGoToPosition(Slider.sliderPos.ZERO_POS);
                this.sleep(200);
            }
            if (this.gamepad1.dpad_left) {
                slider.sliderGoToPosition(Slider.sliderPos.LOW_POS);
                this.sleep(500);
            }
            if (this.gamepad1.dpad_right) {
                slider.sliderGoToPosition(Slider.sliderPos.MID_POS);
                this.sleep(500);
            }
            if (this.gamepad1.dpad_up) {
                slider.sliderGoToPosition(Slider.sliderPos.HIGH_POS);
                this.sleep(500);
            }

            if (this.gamepad1.a) {
                slider.setServoPosition(Slider.servoPos.SERVO_DOWN_POS);
                this.sleep(200);
            }
            if (this.gamepad1.x) {
                slider.setServoPosition(Slider.servoPos.SERVO_MID_POS);
                this.sleep(200);
            }
            if (this.gamepad1.b) {
                slider.setServoPosition(Slider.servoPos.SERVO_UP_POS);
                this.sleep(200);
            }

            if(this.gamepad1.y){
                sus = !sus;
                this.sleep(200);
            }
            if (sus ==  true) {
                manuta.manutaSus();
            } else {
                manuta.manutaJos();
            }


        }


    }
}
