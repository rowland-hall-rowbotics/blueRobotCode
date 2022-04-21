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

    public static final double ONE_CM_IN_PPR = 7.9;

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

    public double calculateWheelCounts(double distanceInCM){
        return distanceInCM * ONE_CM_IN_PPR;
    }

    public void drive(double wCounts, double wheelPower){
        int wheelCounts = 0;

        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while((Math.abs(wheelCounts) < wCounts)){
            wheelCounts = wheels.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                wheels.setPower(wheelPower);
            } else {
                wheels.setPower(0);
            }
        }

        wheels.setPower(0);
    }
}
