package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BasicStateMachineOA", group="Iterative Opmode")
@Disabled
public class BasicStateMachineOA extends LinearOpMode {
    private enumStates state =  enumStates.SEARCHING;
    private DistanceSensor dSensor = null;
    private DcMotor motor = null;
    private Servo servo = null;
    private TouchSensor touch = null;

    private double distance;
    private double robotLength = 34.3;
    private double armLength = 12.7;
    private double reverseSpeed = -.5;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");
        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            if (state == enumStates.SEARCHING) { //Scanning
                distance = dSensor.getDistance(DistanceUnit.CM); //read distance sensor
                if (distance > 0 && distance < (robotLength + armLength)) {//if distance sensor reads greater than 0 less than robotsize + armsize
                    state = enumStates.MAKING_SPACE;//set state to making space
                }
                if (distance > (robotLength + armLength)) {//if distance sensor greater than robotsize + armsize
                    state = enumStates.APPROACHING_WALL;//set state to approaching wall
                }
            } else if (state == enumStates.MAKING_SPACE) {
                distance = dSensor.getDistance(DistanceUnit.CM);
                if (distance < (robotLength + armLength)) {//if distance sensor reads less than robotsize + armsize
                    motor.setPower(reverseSpeed);//set motor power -.5
                } else { //else set motor power 0
                    motor.setPower(0);
                    state = enumStates.SEARCHING;//set state back to scanning
                }
            }
        }
    }


}
