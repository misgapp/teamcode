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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Apollo Teleop", group = "Apollo")
//@Disabled //TODO Enable this to show program in TeleOP list - lasim lev
public class ApolloTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    // could also use HardwarePushbotMatrix class.
    HardwareApollo robot = new HardwareApollo();

    public static final double MID_SERVO = 0.5;
    public static final double liftSpeed = 0.5;

    @Override
    public void runOpMode() {

        double clawDownPosition = 0.17;
        double clawUpPosition = 0.17;
       // double liftPosition = 0.64;
        double left = 0;
        double right = 0;
        double Doobi = 0.2;
        double Mcqueen = 1;
        double bibix = 1;
        Mcqueen = Math.min(Mcqueen, 1);
        Mcqueen = Math.max(Mcqueen, 4);



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        //clawPosition = robot.clawLeft.getPosition();
        //liftPosition = robot.liftLeft.getPosition();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
                bibix = 2;
                idle();
            }

            if (gamepad1.b) {
                bibix = 1;
                idle();
            }

            if (bibix == 1) {
                right = gamepad1.right_stick_y;
                left = gamepad1.left_stick_y;
            }

            if (bibix == 2) {
                right = -gamepad1.right_stick_y;
                left = -gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_up) {
                Doobi = 0.1;
                Mcqueen = Mcqueen - 0.2;
                while (gamepad1.dpad_up) {
                    idle();
                }
                idle();
            }


            if (Doobi == 0.1) {
                robot.leftDriveFront.setPower(left / Mcqueen);
                robot.rightDriveFront.setPower(right / Mcqueen);
                robot.leftDriveBack.setPower(left / Mcqueen);
                robot.rightDriveBack.setPower(right / Mcqueen);
                idle();
            }


            if (gamepad1.dpad_right) {
                Doobi = 0.2;
                idle();
            }

            if (Doobi == 0.2) {
                robot.leftDriveFront.setPower(left / 1.7);
                robot.rightDriveFront.setPower(right / 1.7);
                robot.leftDriveBack.setPower(left / 1.7);
                robot.rightDriveBack.setPower(right / 1.7);
                Mcqueen = 1.7;
                idle();
            }

            if (gamepad1.dpad_left) {
                Doobi = 0.3;
                idle();
            }

            if (Doobi == 0.3) {
                robot.leftDriveFront.setPower(left / 2.5);
                robot.rightDriveFront.setPower(right / 2.5);
                robot.leftDriveBack.setPower(left / 2.5);
                robot.rightDriveBack.setPower(right / 2.5);
                Mcqueen = 2.5;
                idle();
            }


            if (gamepad1.dpad_down) {
                Doobi = 0.4;
                Mcqueen += 0.2;
                while (gamepad1.dpad_down) {
                    idle();
                }
                idle();
            }

            if (Doobi == 0.4) {
                robot.leftDriveFront.setPower(left / Mcqueen);
                robot.rightDriveFront.setPower(right / Mcqueen);
                robot.leftDriveBack.setPower(left / Mcqueen);
                robot.rightDriveBack.setPower(right / Mcqueen);
                idle();
            }


            if (gamepad2.right_trigger > 0) {
                robot.lift.setPower(liftSpeed);
            }

            if (gamepad2.left_trigger > 0) {
                robot.lift.setPower(-liftSpeed);
            }




            double deltaClaw = -gamepad2.left_stick_y;
            double deltaLift = gamepad2.right_stick_y;

            clawDownPosition += deltaClaw;
            clawDownPosition = Math.min(clawDownPosition, 1);
            clawDownPosition = Math.max(clawDownPosition, 0);
            robot.clawDownLeft.setPosition(clawDownPosition);
            robot.clawDownRight.setPosition(1 - clawDownPosition);

            clawUpPosition += deltaLift;
            clawUpPosition = Math.min(clawUpPosition, 1);
            clawUpPosition = Math.max(clawUpPosition, 0);
            robot.clawUpLeft.setPosition(clawUpPosition);
            robot.clawUpRight.setPosition(1 - clawUpPosition);


            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("claw position", "%.2f", robot.clawDownLeft.getPosition());
            telemetry.addData("speed", "%.2f", Mcqueen);
            telemetry.addData("lift position", "%.2f", robot.clawUpLeft.getPosition());
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

}
