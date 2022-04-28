package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HWC {
    Telemetry telemetry;
    public DcMotor wheels;
    public Servo servo;
    public DistanceSensor distanceSensor;
    public TouchSensor button;
    public static double motorPower = 0.0;
    public static final double EXTENDED_ROBOT_LENGTH = 50;
    public static final double ROBOT_LENGTH = 0;

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors and servos
        wheels = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        button = hardwareMap.get(TouchSensor.class, "button");

        wheels.setDirection(DcMotor.Direction.REVERSE);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveToStart(double wheelPower){
        int wheelCounts = wheels.getCurrentPosition();

        wheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(wheelCounts > 0){
            wheelCounts = wheels.getCurrentPosition();

            if(wheelCounts > 0){
                motorPower = wheelPower;
            } else {
                motorPower = 0;
            }
        }

        motorPower = 0;
    }
    public void raiseArm(){ servo.setPosition(0.2); }
}
