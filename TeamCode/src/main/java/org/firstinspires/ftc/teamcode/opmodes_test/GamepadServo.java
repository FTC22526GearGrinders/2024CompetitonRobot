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

package org.firstinspires.ftc.teamcode.opmodes_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Gamepad1 Servo", group = "Concept")
//@Disabled
public class GamepadServo extends LinearOpMode {

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Define class members

    Servo servol;
    Servo servor;
    double position = 0;
    boolean direction = false;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servor = hardwareMap.get(Servo.class, "rightTiltServo");
        servol = hardwareMap.get(Servo.class, "leftTiltServo");

        waitForStart();

        servol.setDirection(Servo.Direction.FORWARD);
        servor.setDirection(Servo.Direction.REVERSE);


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);

//            if (gamepad2.left_bumper && !previousGamepad2.left_bumper)
//                if (servor.getDirection() == Servo.Direction.FORWARD)
//                    servor.setDirection(Servo.Direction.REVERSE);
//                else servor.setDirection(Servo.Direction.FORWARD);

            if (gamepad2.y && !previousGamepad2.y) {
                position = .9;
                servor.setPosition(position);
                servol.setPosition(position);

            }
            if (gamepad2.a && !previousGamepad2.a) {
                position = 0;
                servor.setPosition(position);
                servol.setPosition(position);

            }

            if (gamepad2.x && !previousGamepad2.x) {
                position += .01;
                if (position > 1) position = 1;
                servor.setPosition(position);
                servol.setPosition(position);

            }
            if (gamepad2.b && !previousGamepad2.b) {
                position -= .01;
                if (position < 0) position = 0;
                servor.setPosition(position);
                servol.setPosition(position);
            }


            if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
                position = .5;
                servor.setPosition(position);
                servol.setPosition(position);
            }
            if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
                position = 0;
                servor.setPosition(position);
                servol.setPosition(position);
            }
            if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
                position = .25;
                servor.setPosition(position);
                servol.setPosition(position);
            }
            if (gamepad2.dpad_right && !previousGamepad2.dpad_right) {
                position = .75;
                servor.setPosition(position);
                servol.setPosition(position);
            }



            // Display the current value
            telemetry.addData("Servo Position", position);
            telemetry.addData("Forward", servor.getDirection() == Servo.Direction.FORWARD);
            telemetry.addData("Reverse", servor.getDirection() == Servo.Direction.REVERSE);

            telemetry.update();

            sleep(100);
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
