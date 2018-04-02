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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

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
    private boolean spinDirectionUp = true; // Up is gear wheels are up
    static final double SPIN_SPEED_FAST = -0.6;
    static final double SPIN_SPEED_SLOW = -0.2;
    private float spinAngle = 0;
    boolean isSpinerEnabled = false;
    double spinSpeed = 0;
    boolean manualSpinActive = false;
    boolean gearsAreUp = true;

    @Override
    public void runOpMode() {
        double clawDownPosition = robot.START_POSITION_CLAW_DOWN;
        double clawUpPosition = robot.START_POSITION_CLAW_UP;
        double speed_Left = 0;
        double speed_Right = 0;
        double driveSpeedFactor = SPEED_FACTOR_1;
        boolean speedFactorUpPressHandled = false;
        boolean speedFactorDownPressHandled = false;
        boolean clawRelic = true;
        boolean gamepad2_bumper_previous_pressed = false;
        boolean balltaskisup = false;



        robot.init(hardwareMap);

        robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (balltaskisup == false) {
                robot.armRightLeft.setPosition(0.38);
                sleep(500);
                robot.armUpDown.setPosition(0.15);
                balltaskisup = true;
            }

            speed_Left = -gamepad1.left_stick_y;
            speed_Right = -gamepad1.right_stick_y;

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
                robot.setPowerLifts(LIFT_SPEED);
            } else if (gamepad2.left_trigger > 0) {
                robot.setPowerLifts(-LIFT_SPEED);
            } else {
                robot.setPowerLifts(0);
            }

            //Set position to claws according to the sticks
            double deltaClawDown = spinDirectionUp ? -gamepad2.left_stick_y / 2 : -gamepad2.right_stick_y / 2;

            if (deltaClawDown < 0.3 && deltaClawDown > -0.3) {
                deltaClawDown = 0;
            }

            double deltaClawUp = spinDirectionUp ? -gamepad2.right_stick_y / 2 : gamepad2.left_stick_y / 2;

            if (deltaClawUp < 0.3 && deltaClawUp > -0.3) {
                deltaClawUp = 0;
                telemetry.addData("deltaClawUp - clear", "%.2f", deltaClawUp);
            }

            clawDownPosition += deltaClawDown;
            clawDownPosition = Math.min(clawDownPosition, 0.45);
            clawDownPosition = Math.max(clawDownPosition, 0.3);
            robot.clawDownLeft.setPosition(clawDownPosition);
            robot.clawDownRight.setPosition(1 - clawDownPosition);

            clawUpPosition += deltaClawUp;
            clawUpPosition = Math.min(clawUpPosition, 0.7);
            clawUpPosition = Math.max(clawUpPosition, 0.55);
            robot.clawUpLeft.setPosition(clawUpPosition);
            robot.clawUpRight.setPosition(1 - clawUpPosition);


            /*
            if (spinDirectionUp == true) {
                //Set position to claws according to the sticks
                double deltaClawDown = -gamepad2.right_stick_y / 2;

                if (deltaClawDown < 0.3 && deltaClawDown > -0.3) {
                    deltaClawDown = 0;
                }

                double deltaClawUp = gamepad2.left_stick_y / 2;

                if (deltaClawUp < 0.3 && deltaClawUp > -0.3) {
                    deltaClawUp = 0;
                    telemetry.addData("deltaClawUp - clear", "%.2f", deltaClawUp);
                }

                clawDownPosition += deltaClawDown;
                clawDownPosition = Math.min(clawDownPosition, 0.45);
                clawDownPosition = Math.max(clawDownPosition, 0.3);
                robot.clawDownLeft.setPosition(clawDownPosition);
                robot.clawDownRight.setPosition(1 - clawDownPosition);

                clawUpPosition += deltaClawUp;
                clawUpPosition = Math.min(clawUpPosition, 0.7);
                clawUpPosition = Math.max(clawUpPosition, 0.55);
                robot.clawUpLeft.setPosition(clawUpPosition);
                robot.clawUpRight.setPosition(1 - clawUpPosition);
            }else {
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
                clawDownPosition = Math.min(clawDownPosition, 0.45);
                clawDownPosition = Math.max(clawDownPosition, 0.3);
                robot.clawDownLeft.setPosition(clawDownPosition);
                robot.clawDownRight.setPosition(1 - clawDownPosition);

                clawUpPosition += deltaClawUp;
                clawUpPosition = Math.min(clawUpPosition, 0.7);
                clawUpPosition = Math.max(clawUpPosition, 0.55);
                robot.clawUpLeft.setPosition(clawUpPosition);
                robot.clawUpRight.setPosition(1 - clawUpPosition);
            } */

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
                robot.relicUpDown.setPosition(0.53);
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

            if (gamepad2.dpad_left) {
                robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.spinner.setPower(-0.25);
                isSpinerEnabled = false;
                manualSpinActive = true;
            } else if (gamepad2.dpad_right) {
                robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.spinner.setPower(0.25);
                isSpinerEnabled = false;
                manualSpinActive = true;
            } else if (manualSpinActive) {
                robot.spinner.setPower(0);
                manualSpinActive = false;
            }

            //Change the direction of the spin
            if (gamepad1.y) {
                if (!isSpinerEnabled) {
                    isSpinerEnabled = true;
                    robot.setPositionClaw(0.7, 0.3);
                    spinDirectionUp = !spinDirectionUp;
                    spinSpeed = spinDirectionUp ? -0.7 : 0.7;
                    robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.spinner.setPower(spinSpeed);
                }
                isSpinerEnabled = true;
            }

            //Set and change power to spinner according to gyro angle
            if (isSpinerEnabled) {
                spin();
            }

            telemetry.addData("tick", "%d", robot.liftLeft.getCurrentPosition());
            telemetry.addData("claw Down Left", "%.2f", robot.clawDownLeft.getPosition());
            telemetry.addData("claw Down Right", "%.2f", robot.clawDownRight.getPosition());
            telemetry.addData("claw up Left", "%.2f", robot.clawUpLeft.getPosition());
            telemetry.addData("claw up Right", "%.2f", robot.clawUpRight.getPosition());
            telemetry.addData("Spin angle", "%.2f", spinAngle);
            telemetry.addData("spinner enabled", isSpinerEnabled);
            telemetry.addData("spinDirectionUp", spinDirectionUp);
            telemetry.addData("touch up", robot.touchSpinnerUp.getState());
            telemetry.addData("touch down", robot.touchSpinnerDown.getState());
            telemetry.addData("spinner ticks", "%d", robot.spinner.getCurrentPosition());
            telemetry.addData("dictance sensor ", "%.2f", robot.sensorDistanceDown.getDistance(DistanceUnit.CM));
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    public void spin() {
        if ((!spinDirectionUp && !robot.touchSpinnerDown.getState()) ||
                (spinDirectionUp && !robot.touchSpinnerUp.getState())) {
            robot.spinner.setPower(0);
            isSpinerEnabled = false;
        }
    }

    /*
    //Set and change power to spinner according to gyro angle
    public void spin1() {
        spinAngle = robot.gyroSpinner.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (spinDirectionUp) {
            if (spinAngle <= 90 && spinAngle >= 25) {
                robot.spinner.setPower(SPIN_SPEED_FAST);
            } else if (spinAngle < 25 && spinAngle >= 4) {
                robot.spinner.setPower(SPIN_SPEED_SLOW);
            } else if ((spinAngle < 4 && spinAngle >= 0) || (spinAngle >= -4 && spinAngle < 0)) {
                robot.spinner.setPower(0);
            } else if (spinAngle < -4 && spinAngle >= -45) {
                robot.spinner.setPower(-SPIN_SPEED_SLOW);
            } else {
                robot.spinner.setPower(-SPIN_SPEED_FAST);
            }
        } else {
            if (spinAngle >= 90 && spinAngle <= 155) {
                robot.spinner.setPower(-SPIN_SPEED_FAST);
            } else if (spinAngle <= 176 && spinAngle > 155) {
                robot.spinner.setPower(-SPIN_SPEED_SLOW);
            } else if (spinAngle <= -176 || spinAngle > 176) {
                robot.spinner.setPower(0);
            } else if (spinAngle <= -140 && spinAngle > -176) {
                robot.spinner.setPower(SPIN_SPEED_SLOW);
            } else {
                robot.spinner.setPower(SPIN_SPEED_FAST);
            }
        }
    }
    */
}
