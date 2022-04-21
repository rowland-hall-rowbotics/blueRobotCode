package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class buttonBot extends LinearOpMode {
    boolean buttonPushed = false;

    public enum enumStates {
        SEARCHING,
        APPROACHING_WALL,
        MAKING_SPACE,
        RESETTING,
        LOWERING
    }
    enumStates state;

    @Override
    public void runOpMode() throws InterruptedException {
        // All states: Searching, Moving Forward, Arm Down, Moving Backwards, Returning Home, Button Pushed, Arm Up
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        HWC robot = new HWC(hardwareMap, telemetry);

        if(robot.servo.getPosition() == 0.2){
            return;
        } else {
            robot.servo.setPosition(0.2);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(state == enumStates.RESETTING){
            // When we drove back, we subtracted tha amount of ticks driven backwards from the distanceDriven Variable.
            // When we drove forward, we add the amount of ticks driven forward to the distanceDriven Variable
            // get the distanceDriven, and drive backwards that many ticks.
            // If there is not enough space to close the arm from the starting position, make room.

            robot.drive(robot.calculateWheelCounts(50), 0.5);
        }
    }


}
