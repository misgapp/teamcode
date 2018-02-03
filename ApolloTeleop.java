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

//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * This OpMode uses the common Apollo hardware class to define the devices on the robot.
 * All device access is managed through the TestHardwareSingleMotor class.
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

    static final double LIFT_SPEED = 0.9;
    static final double SPEED_FACTOR_1 = 1.0;
    static final double SPEED_FACTOR_2 = 1.4;
    static final double SPEED_FACTOR_3 = 2.0;
    static final double SPEED_FACTOR_4 = 2.5;

    @Override
    public void runOpMode() {

        double clawDownPosition = robot.START_POSITION_CLAW_DOWN;
        double clawUpPosition = robot.START_POSITION_CLAW_UP;
        double speed_Left = 0;
        double speed_Right = 0;
        double driveSpeedFactor = SPEED_FACTOR_1;
        boolean driveDirectionForward = false;
        boolean speedFactorUpPressHandled = false;
        boolean speedFactorDownPressHandled = false;
        boolean armRelic = true;
        boolean clawRelic = true;
        boolean gamepad2_x_previous_pressed = false;
        boolean gamepad2_a_previous_pressed = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.clawRoll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", "1");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Change speed of the motors
            if (gamepad1.x) {
                driveDirectionForward = true;
                idle();
            }

            if (gamepad1.b) {

                driveDirectionForward = false;
                idle();
            }

            if (driveDirectionForward) {
                speed_Left = gamepad1.right_stick_y;
                speed_Right = gamepad1.left_stick_y;
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

            //Set power to the motor according to the change
            robot.driveBackLeft.setPower(speed_Left / driveSpeedFactor);
            robot.driveBackRight.setPower(speed_Right / driveSpeedFactor);
            robot.driveFrontLeft.setPower(speed_Left / driveSpeedFactor);
            robot.driveFrontRight.setPower(speed_Right/ driveSpeedFactor);

            //Set power to the lift according to the buttons
            if (gamepad2.right_trigger > 0 /*&& 10000 > robot.lift.getCurrentPosition() && robot.lift.getCurrentPosition() > 500*/) {
                robot.lift.setPower(LIFT_SPEED);
            } else if (gamepad2.left_trigger > 0/*&& 10000 > robot.lift.getCurrentPosition() && robot.lift.getCurrentPosition() > 500*/) {
                robot.lift.setPower(-LIFT_SPEED);
            } else {
                robot.lift.setPower(0);
            }

            double deltaClawDown = -gamepad2.left_stick_y / 2;

            if (deltaClawDown < 0.3 && deltaClawDown > -0.3) {
                deltaClawDown = 0;
            }

            double deltaClawUp = gamepad2.right_stick_y / 2;

            if (deltaClawUp < 0.3 && deltaClawUp > -0.3) {
                deltaClawUp = 0;
                telemetry.addData("deltaClawUp - clear", "%.2f", deltaClawUp);
            }

            clawDownPosition += deltaClawDown;
            clawDownPosition = Math.min(clawDownPosition, 0.7);
            clawDownPosition = Math.max(clawDownPosition, 0.5);
            robot.clawDownLeft.setPosition(clawDownPosition);
            robot.clawDownRight.setPosition(1 - clawDownPosition);

            clawUpPosition += deltaClawUp;
            clawUpPosition = Math.min(clawUpPosition, 0.7);
            clawUpPosition = Math.max(clawUpPosition, 0.5);
            robot.clawUpLeft.setPosition(clawUpPosition);
            robot.clawUpRight.setPosition(1 - clawUpPosition);

            if (gamepad1.left_trigger > 0) {
                robot.setPositionWheel(robot.DROP_POSITION);
            } else if (gamepad1.right_trigger > 0) {
                robot.setPositionWheel(robot.GRAB_POSITION);
            } else {
                robot.setPositionWheel(robot.STOP_POSITION);
            }

            /*
            if (gamepad1.left_bumper){
                encoderRoll(0.3, 300);
            } else if (gamepad1.right_bumper){
                encoderRoll(0.3, -300);
            }

            if (gamepad2.b){
                encoderRoll(0.3, 50);
            } else if (gamepad2.y){
                encoderRoll(0.3, -50);
            }
            */

            if (gamepad2.y){
                robot.relicUpDown.setPosition(0.45);
            }

            if (gamepad2.x){
                robot.relicUpDown.setPosition(0.2);
            }

            if (gamepad2.a){
                robot.relicClaw.setPosition(0.4);
            }


            if (gamepad2.x){
                if (!gamepad2_x_previous_pressed) {
                    gamepad2_x_previous_pressed = true;
                    if (armRelic) {
                        robot.relicUpDown.setPosition(0.1);
                        armRelic = false;
                    } else {
                        robot.relicUpDown.setPosition(0.9);
                        armRelic = true;
                    }
                }
            } else {
                gamepad2_x_previous_pressed = false;
            }



            if (gamepad2.a) {
                if (!gamepad2_a_previous_pressed) {
                    gamepad2_a_previous_pressed = true;
                    if (clawRelic){
                        robot.relicClaw.setPosition(0.05);
                        clawRelic = false;
                    } else {
                        robot.relicClaw.setPosition(0.4);
                        clawRelic = true;
                    }
                }
            } else {
                gamepad2_a_previous_pressed = false;
            }


            if (gamepad2.dpad_down){
                robot.relicLift.setPower(1);
            } else if (gamepad2.dpad_up){
                robot.relicLift.setPower(-1);
            } else {
                robot.relicLift.setPower(0);
            }

/*

            telemetry.addData("left", "%.2f", speed_Left);
            telemetry.addData("right", "%.2f", speed_Right);
            telemetry.addData("speed", "%.2f", driveSpeedFactor);
            telemetry.addData("claw down position", "%.2f", robot.clawDownLeft.getPosition());
            //telemetry.addData("claw up position", "%.2f", robot.clawUpLeft.getPosition());
            //telemetry.addData("arm Right Left", "%.2f", robot.armRightLeft.getPosition());
            //telemetry.addData("relic Claw", "%.2f", robot.relicClaw.getPosition());
            //telemetry.addData("relic Arm", "%.2f", robot.relicArm.getPosition());
            //telemetry.addData("arm up Down", "%.2f", robot.armUpDown.getPosition());
            telemetry.addData("deltaClawUp", "%.2f", deltaClawUp);
            telemetry.addData("drive speed factor", "%.2f", driveSpeedFactor);
            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            //telemetry.addData("drive direction forward", "%.2f", driveDirectionForward);
            telemetry.update();
*/
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    public void encoderRoll(double speed, int tick) {
        robot.clawRoll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawRoll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget = 0;

        speed = Math.abs(speed);
        double speedMotor = tick > 0 ? speed : -speed;

        newTarget = robot.clawRoll.getCurrentPosition() + tick;

        robot.clawRoll.setPower(speedMotor);


        while (opModeIsActive()) {
            if (tick > 0) {
                if (robot.clawRoll.getCurrentPosition() >= newTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.clawRoll.getCurrentPosition() <= newTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }

            telemetry.addData("tick lift", "%d", robot.clawRoll.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.clawRoll.setPower(0);
    }

    }
