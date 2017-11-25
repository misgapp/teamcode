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

/**
 * This OpMode uses the common Apollo hardware class to define the devices on the robot.
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
    // the lift is in Remarks because we dont have another  motor
    HardwareApollo robot = new HardwareApollo();

    static final double LIFT_SPEED = 0.45;
    static final double SPEED_FACTOR_1 = 1.2;
    static final double SPEED_FACTOR_2 = 1.8;
    static final double SPEED_FACTOR_3 = 2.6;
    static final double SPEED_FACTOR_4 = 3.5;

    @Override
    public void runOpMode() {

        double clawDownPosition = 0.9;
        double clawUpPosition = 0.1;
        double speed_Left = 0;
        double speed_Right = 0;
        double driveSpeedFactor = SPEED_FACTOR_3;
        boolean driveDirectionForward = true;
        boolean speedFactorUpPressHandled = false;
        boolean speedFactorDownPressHandled = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Go Go Apollo");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
                driveDirectionForward = true;
                idle();
            }

            if (gamepad1.b) {
                driveDirectionForward = false;
                idle();
            }

            if (driveDirectionForward) {
                speed_Right = gamepad1.right_stick_y;
                speed_Left = gamepad1.left_stick_y;
            } else {
                speed_Right = -gamepad1.right_stick_y;
                speed_Left = -gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_up) {
                if (!speedFactorUpPressHandled) {
                    speedFactorUpPressHandled = true;
                    if (driveSpeedFactor == SPEED_FACTOR_4) {
                        driveSpeedFactor = SPEED_FACTOR_3;
                    } else if (driveSpeedFactor == SPEED_FACTOR_3) {
                        driveSpeedFactor = SPEED_FACTOR_2;
                    } else if (driveSpeedFactor == SPEED_FACTOR_2) {
                        driveSpeedFactor = SPEED_FACTOR_1;
                    }
                }
            } else {
                speedFactorUpPressHandled = false;
            }

            if (gamepad1.dpad_down) {
                if (!speedFactorDownPressHandled) {
                    speedFactorDownPressHandled = true;
                    if (driveSpeedFactor == SPEED_FACTOR_1) {
                        driveSpeedFactor = SPEED_FACTOR_2;
                    } else if (driveSpeedFactor == SPEED_FACTOR_2) {
                        driveSpeedFactor = SPEED_FACTOR_3;
                    } else if (driveSpeedFactor == SPEED_FACTOR_3) {
                        driveSpeedFactor = SPEED_FACTOR_4;
                    }
                }
            } else {
                speedFactorDownPressHandled = false;
            }

            robot.setPowerAllDriveMotors(speed_Left / driveSpeedFactor);

            if (gamepad2.left_trigger > 0) {
                robot.lift.setPower(LIFT_SPEED);
            } else if (gamepad2.right_trigger > 0) {
                robot.lift.setPower(-LIFT_SPEED);
            } else {
                robot.lift.setPower(0);
            }

            double deltaClawDown = -gamepad2.left_stick_y;
            if (deltaClawDown < 0.3 && deltaClawDown > -0.3) {
                deltaClawDown = 0;
            }
            double deltaClawUp = gamepad2.right_stick_y;
            if (deltaClawUp < 0.3 && deltaClawUp > -0.3) {
                deltaClawUp = 0;
                telemetry.addData("deltaClawUp - clear", "%.2f", deltaClawUp);
            }

            clawDownPosition += deltaClawDown;
            clawDownPosition = Math.min(clawDownPosition, 1);
            clawDownPosition = Math.max(clawDownPosition, 0);
            robot.clawDownLeft.setPosition(clawDownPosition);
            robot.clawDownRight.setPosition(1 - clawDownPosition);

            clawUpPosition += deltaClawUp;
            clawUpPosition = Math.min(clawUpPosition, 1);
            clawUpPosition = Math.max(clawUpPosition, 0);
            robot.clawUpLeft.setPosition(clawUpPosition);
            robot.clawUpRight.setPosition(1 - clawUpPosition);

            telemetry.addData("left", "%.2f", speed_Left);
            telemetry.addData("right", "%.2f", speed_Right);
            telemetry.addData("claw down position", "%.2f", robot.clawDownLeft.getPosition());
            telemetry.addData("speed", "%.2f", driveSpeedFactor);
            telemetry.addData("claw up position", "%.2f", robot.clawUpLeft.getPosition());
            telemetry.addData("deltaClawUp", "%.2f", deltaClawUp);
            telemetry.addData("drive speed factor", "%.2f", driveSpeedFactor);
         // telemetry.addData("drive direction forward", "%.2f", driveDirectionForward);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
