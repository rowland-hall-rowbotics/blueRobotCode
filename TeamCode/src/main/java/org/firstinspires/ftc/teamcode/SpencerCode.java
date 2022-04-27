/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class SpencerCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;
    private DistanceSensor distanceSensor = null;
    private Servo servo = null;
    private int robotLength = 0;
    private TouchSensor button = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor  = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class,"servo");
        servo.setPosition(0);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        button = hardwareMap.get(TouchSensor.class,"button");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double motorPower;
        enumStates state = null;
        servo.setPosition(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorPower = 0;
            if(state == enumStates.SEARCHING) {
                //Scan for wall

                //If wall is sensed - move to DRIVEANDLOWER
                if(distanceSensor.getDistance(DistanceUnit.CM) > 0){
                    state = enumStates.APPROACHING_WALL;
                }
            } else if(state == enumStates.APPROACHING_WALL){
                //Drive forward and lower arm
                motorPower = 1;
                servo.setPosition(1);
                //If closer than distance from distance sensor to fully extended touch sensor - move to LOWER

                //If touch sensor pressed - move to BACKWARDANDRAISING
                if(distanceSensor.getDistance(DistanceUnit.CM) < robotLength && servo.getPosition() == 1){
                    state = enumStates.LOWERING;
                }
                if(button.isPressed()) {
                    state = enumStates.RESETTING;
                }
                //might be affected by the arm triggering the ultrasonic sensor
            } else if(state == enumStates.LOWERING){
                //Lower arm
                servo.setPosition(1);
                //If wall is too close to robot for arm to lower - move to BACKUPTOLOWER
                if(distanceSensor.getDistance(DistanceUnit.CM) < robotLength - 5){
                    state = enumStates.MAKING_SPACE;
                }
                //If arm fully lowered - move to FORWARD
                if(servo.getPosition() == 1) {
                    state = enumStates.APPROACHING_WALL;
                }
            } else if(state == enumStates.MAKING_SPACE){
                //Back up to be able to lower arm
                motorPower = -1;
                //If backed up far enough - move to LOWER
                if(distanceSensor.getDistance(DistanceUnit.CM) < robotLength){
                    state = enumStates.LOWERING;
                }

            } else if(state == enumStates.RESETTING){
                //Drive backward and retract arm
                servo.setPosition(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (motor.getCurrentPosition() < 0 && servo.getPosition() <= 0.5) {
                    motor.setTargetPosition(0);
                }

                if(motor.getCurrentPosition() == 0 && servo.getPosition() == 0){
                    state = enumStates.SEARCHING;
                }


                //Back up to be able to lower arm

                //Drive forward to parked position

                //If at target position and arm raised - move to SCANNING
            } else {
                //do nothing
            }



            // Send calculated power to wheels
            motor.setPower(motorPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "motor (%.2f)", motorPower);
            telemetry.addData("Distance", "distace (%.2f)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Servo", "servo (%.2f)", servo.getPosition());
            telemetry.update();
        }
    }
}
