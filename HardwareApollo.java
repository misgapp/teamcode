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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// the lift is in Remarks because we dont have another  motor

public class HardwareApollo {
    /* Public OpMode members. */
    public DcMotor driveFrontLeft = null;
    public DcMotor driveFrontRight = null;
    public DcMotor driveBackLeft = null;
    public DcMotor driveBackRight = null;
    public Servo clawDownLeft = null;
    public Servo clawDownRight = null;
    public Servo clawUpLeft = null;
    public Servo clawUpRight = null;
    public DcMotor lift = null;
    // public ColorSensor sensor_color = null;
    // public TouchSensor sensor_button = null;
    public static final double start_Position_clawUp = 0.1;
    public static final double start_Position_clawDown = 0.9;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        driveBackLeft = hwMap.get(DcMotor.class, "drive_back_left");
        driveBackRight = hwMap.get(DcMotor.class, "drive_back_right");
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        driveFrontLeft = hwMap.get(DcMotor.class, "drive_front_left");
        driveFrontRight = hwMap.get(DcMotor.class, "drive_front_right");
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        driveFrontLeft.setPower(0);
        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        lift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawDownLeft = hwMap.get(Servo.class, "claw_down_left");
        clawDownRight = hwMap.get(Servo.class, "claw_down_right");
        clawUpLeft = hwMap.get(Servo.class, "claw_up_left");
        clawUpRight = hwMap.get(Servo.class, "claw_up_right");

        clawDownLeft.setPosition(start_Position_clawDown);
        clawDownRight.setPosition(start_Position_clawDown);
        clawUpLeft.setPosition(start_Position_clawUp);
        clawUpRight.setPosition(start_Position_clawUp);
        /*
        sensor_color = hwMap.get(ColorSensor.class, "sensor_color");
        sensor_button = hwMap.get(TouchSensor.class, "sensor_button");
        */
    }
}

