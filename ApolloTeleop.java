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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.AutoMain.HEADING_THRESHOLD;

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

    static final double LIFT_SPEED = 1;
    static final double SPEED_FACTOR_1 = 1.0;
    static final double SPEED_FACTOR_2 = 1.3;
    static final double SPEED_FACTOR_3 = 1.6;
    static final double SPEED_FACTOR_4 = 2.0;
    private static final double P_TURN_COEFF = 0.1;
    private boolean spinDirectionUp = true; // Up is gear wheels are up
    static final double SPIN_SPEED_FAST = -0.6;
    static final double SPIN_SPEED_SLOW = -0.35;
    private float spinAngle = 0;

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
        boolean gamepad2_bumper_previous_pressed = false;
        boolean isSpinerPressed = false;
        boolean isSpinerEnabled = true;
        int angleClaws = 0;
        ElapsedTime timer = new ElapsedTime();
        //double angleClaws = 0;


        robot.init(hardwareMap);

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyroSpiner.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyroSpiner.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.clawRoll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", "1");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.gyroSpiner.resetZAxisIntegrator();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.armRightLeft.setPosition(0.38);
            robot.armUpDown.setPosition(0.32);


            //Change speed of the motors
            if (gamepad1.dpad_left) {
                driveDirectionForward = true;
                idle();
            }

            if (gamepad1.dpad_right) {
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

            //Change variables to change velocity
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
            robot.driveFrontRight.setPower(speed_Right / driveSpeedFactor);

            //Set power to the lift according to the buttons
            if (gamepad2.right_trigger > 0) {
                robot.lift.setPower(LIFT_SPEED);
            } else if (gamepad2.left_trigger > 0) {
                robot.lift.setPower(-LIFT_SPEED);
            } else {
                robot.lift.setPower(0);
            }

            //Set position to claws according to the sticks
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
            clawDownPosition = Math.max(clawDownPosition, 0.3);
            robot.clawDownLeft.setPosition(clawDownPosition);
            robot.clawDownRight.setPosition(1 - clawDownPosition);

            clawUpPosition += deltaClawUp;
            clawUpPosition = Math.min(clawUpPosition, 0.7);
            clawUpPosition = Math.max(clawUpPosition, 0.3);
            robot.clawUpLeft.setPosition(clawUpPosition);
            robot.clawUpRight.setPosition(1 - clawUpPosition);

            // Set position to the wheels DROP, GRAB or STOP
            if (gamepad1.left_trigger > 0) {
                robot.setPositionWheel(robot.DROP_POSITION);
            } else if (gamepad1.right_trigger > 0) {
                robot.setPositionWheel(robot.GRAB_POSITION);
            } else {
                robot.setPositionWheel(robot.STOP_POSITION);
            }

            //Set position to relic servo
            if (gamepad2.y) {
                robot.relicUpDown.setPosition(0.0);
            }

            if (gamepad2.a) {
                robot.relicUpDown.setPosition(0.6);
            }

            if (gamepad2.b) {
                robot.relicUpDown.setPosition(0.45);
            }

            if (gamepad2.x) {
                robot.relicUpDown.setPosition(0.3);
            }

            //Set position to relic claw
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                if (!gamepad2_bumper_previous_pressed) {
                    gamepad2_bumper_previous_pressed = true;
                    if (clawRelic) {
                        robot.relicClaw.setPosition(0.15);
                        clawRelic = false;
                    } else {
                        robot.relicClaw.setPosition(0.5);
                        clawRelic = true;
                    }
                }
            } else {
                gamepad2_bumper_previous_pressed = false;
            }

            // Set power to relic lift
            if (gamepad2.dpad_down) {
                robot.relicLift.setPower(1);
            } else if (gamepad2.dpad_up) {
                robot.relicLift.setPower(-1);
            } else {
                robot.relicLift.setPower(0);
            }
/*
            //Set power to spiner according to gyro
            if (gamepad1.y) {
                if (!isSpinered){
                    robot.setPositionClaw(0.7, 0.3);
                    encoderDriveSpiner(0.3, 640);
                    isSpinered = true;
                } else {
                    robot.setPositionClaw(0.7, 0.3);
                    encoderDriveSpiner(0.3, -640);
                    isSpinered = false;
                }
            }
            */

            if (gamepad2.dpad_left){
                robot.spiner.setPower(0.35);
                isSpinerEnabled = false;
            } else if (gamepad2.dpad_right){
                robot.spiner.setPower(-0.35);
                isSpinerEnabled = false;
            } else {
                robot.spiner.setPower(0);
            }

            //Change the direction of the spin
            if (gamepad1.y) {
                if (!isSpinerPressed){
                    robot.setPositionClaw(0.7, 0.3);
                    spinDirectionUp = !spinDirectionUp;
                    isSpinerPressed = true;
                }
                isSpinerEnabled = true;
            } else {
                isSpinerPressed = false;
            }

            //Set and change power to spinner according to gyro angle
            if (isSpinerEnabled) {
                spin();
            }

            telemetry.addData("claw Down Left", "%.2f", robot.clawDownLeft.getPosition());
            telemetry.addData("claw Down Right", "%.2f", robot.clawDownRight.getPosition());
            telemetry.addData("claw up Left", "%.2f", robot.clawUpLeft.getPosition());
            telemetry.addData("claw up Right", "%.2f", robot.clawUpRight.getPosition());
            telemetry.addData("Spin angle", "%.2f", spinAngle);
            telemetry.addData("spinDirectionUp", spinDirectionUp);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);

        }
    }

    //Set and change power to spinner according to gyro angle
    public void spin(){
        spinAngle = robot.gyroSpiner.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (spinDirectionUp){
            if (spinAngle <= 90 && spinAngle >= 25){
                robot.spiner.setPower(SPIN_SPEED_FAST);
            } else if (spinAngle < 25 && spinAngle >= 6){
                robot.spiner.setPower(SPIN_SPEED_SLOW);
            } else if ((spinAngle < 6 && spinAngle >= 0) || (spinAngle >= -6 && spinAngle < 0)){
                robot.spiner.setPower(0);
            } else if (spinAngle < -6 && spinAngle >= -45){
                robot.spiner.setPower(-SPIN_SPEED_SLOW);
            } else {
                robot.spiner.setPower(-SPIN_SPEED_FAST);
            }
        } else {
            if (spinAngle >= 90 && spinAngle <= 155){
                robot.spiner.setPower(-SPIN_SPEED_FAST);
            } else if (spinAngle <= 174 && spinAngle > 155){
                robot.spiner.setPower(-SPIN_SPEED_SLOW);
            } else if (spinAngle <= -174 || spinAngle > 174){
                robot.spiner.setPower(0);
            } else if (spinAngle <= -140 && spinAngle > -174){
                robot.spiner.setPower(SPIN_SPEED_SLOW);
            } else {
                robot.spiner.setPower(SPIN_SPEED_FAST);
            }
        }
    }

    // Function drive encoder to spiner
    public void encoderDriveSpiner(double speed, int tick) {
        robot.spiner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spiner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        speed = Math.abs(speed);
        double Speed = tick > 0 ? speed : -speed;

        newLeftTarget = robot.spiner.getCurrentPosition() + tick;

        robot.spiner.setPower(Speed);

        while (opModeIsActive()) {
            if (tick > 0) {
                if (robot.spiner.getCurrentPosition() >= newLeftTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.spiner.getCurrentPosition() <= newLeftTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }

            telemetry.addData("tick lift", "%d", robot.spiner.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.spiner.setPower(0);
    }
}


