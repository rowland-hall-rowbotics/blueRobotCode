package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class buttonBot extends LinearOpMode {
    boolean buttonPushed = false;
    double distanceDriven = 0.0;
    double motorPower = 0.0;
    enumStates state;

    @Override
    public void runOpMode() throws InterruptedException {
        HWC robot = new HWC(hardwareMap, telemetry);

        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.servo.setPosition(0.2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            if (state == enumStates.SEARCHING) {
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 0 && robot.distanceSensor.getDistance(DistanceUnit.CM) < 50) {
                    state = enumStates.MAKING_SPACE;
                }
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) > (HWC.EXTENDED_ROBOT_LENGTH)) {
                    state = enumStates.APPROACHING_WALL;
                }
            } else if (state == enumStates.APPROACHING_WALL) {
                //Drive forward and lower arm
                motorPower = 1;
                robot.servo.setPosition(0.8);
                //If closer than distance from distance sensor to fully extended touch sensor - move to LOWER

                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < HWC.ROBOT_LENGTH && robot.servo.getPosition() != 1) {
                    state = enumStates.LOWERING;
                }
                if (robot.button.isPressed()) {
                    state = enumStates.RESETTING;
                }
                //might be affected by the arm triggering the ultrasonic sensor
            } else if (state == enumStates.LOWERING) {
                //Lower arm
                robot.servo.setPosition(1);
                //If wall is too close to robot for arm to lower - move to BACKUP_TO_LOWER
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < HWC.ROBOT_LENGTH - 5) {
                    state = enumStates.MAKING_SPACE;
                }
                //If arm fully lowered - move to FORWARD
                if (robot.servo.getPosition() == 1) {
                    state = enumStates.APPROACHING_WALL;
                }
            } else if (state == enumStates.MAKING_SPACE) {
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < HWC.EXTENDED_ROBOT_LENGTH) {//if distance sensor reads less than robotsize + armsize
                    motorPower = -0.5;//set motor power -.5
                } else { //else set motor power 0
                    motorPower = 0;
                    state = enumStates.SEARCHING;//set state back to scanning
                }
            } else if (state == enumStates.RESETTING){
                robot.raiseArm();
                robot.driveToStart(0.5);
            }

            robot.wheels.setPower(motorPower);
        }
    }


}
