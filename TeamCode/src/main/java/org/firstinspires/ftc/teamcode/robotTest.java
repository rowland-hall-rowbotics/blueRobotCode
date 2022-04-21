package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class robotTest extends LinearOpMode {
    HWC robot = new HWC(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry.addData("Distance", robot.dSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Servo Position", robot.servo.getPosition());
        telemetry.update();
    }
}