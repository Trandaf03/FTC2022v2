package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.util.breakingModeUtil;
import org.firstinspires.ftc.teamcode.util.directionUtil;
import org.firstinspires.ftc.teamcode.util.encoderUtil;
import org.firstinspires.ftc.teamcode.util.odometryUtil;


@Autonomous (name = "autonomie")
public class Autonomie extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, breakingModeUtil.breakingMode.BRAKE, directionUtil.direction.FORWARD, encoderUtil.encoderMode.RUN_USING);
        odometryUtil odo = new odometryUtil(hardwareMap,drive,telemetry);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("teapa nu am autonomie");
            telemetry.update();
        }

    }
}
