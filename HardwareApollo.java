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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
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
    public DcMotor lift = null;
    public Servo clawDownLeft = null;
    public Servo clawDownRight = null;
    public Servo clawUpLeft = null;
    public Servo clawUpRight = null;
    public Servo armUpDown = null;
    public Servo armRightLeft = null;
    public Servo relicArm = null;
    public Servo relicClaw = null;
    public TouchSensor sensorTouch = null;
    //public ColorSensor sensorColor = null;
    public I2cAddr colorAddr = I2cAddr.create8bit(0x3c);
    public I2cDevice color = null;
    public I2cDeviceSynch colorReader = null;

    public TouchSensor sensor_button = null;

    public static final double start_Position_clawUp = 0.5;
    public static final double start_Position_clawDown = 0.5;
    public static final double start_Position_armUpDown = 0.75;
    public static final double start_Position_armRightLeft = 0.1;
    public static final double start_Position_relicClaw = 0.5;
    public static final double start_Position_relicArm = 0.5;

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
        driveFrontLeft = hwMap.get(DcMotor.class, "drive_front_left");
        driveFrontRight = hwMap.get(DcMotor.class, "drive_front_right");
        lift = hwMap.get(DcMotor.class, "lift");

        driveBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        lift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        setPowerAllDriveMotors(0);
        lift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawDownLeft = hwMap.get(Servo.class, "claw_down_left");
        clawDownRight = hwMap.get(Servo.class, "claw_down_right");
        clawUpLeft = hwMap.get(Servo.class, "claw_up_left");
        clawUpRight = hwMap.get(Servo.class, "claw_up_right");
        armRightLeft = hwMap.get(Servo.class, "arm_right_left");
        armUpDown = hwMap.get(Servo.class, "arm_up_down");
        //relicArm = hwMap.get(Servo.class, "relic_arm");
        //relicClaw = hwMap.get(Servo.class, "relic_claw");

        clawDownLeft.setPosition(start_Position_clawDown);
        clawDownRight.setPosition(start_Position_clawDown);
        clawUpLeft.setPosition(start_Position_clawUp);
        clawUpRight.setPosition(start_Position_clawUp);
        armUpDown.setPosition(start_Position_armUpDown);
        armRightLeft.setPosition(start_Position_armRightLeft);
        //relicArm.setPosition(start_Position_relicArm);
        //relicClaw.setPosition(start_Position_relicClaw);


        //sensorColor = hwMap.get(ColorSensor.class, "sensor_color");
        color = hwMap.i2cDevice.get("sc");
        colorReader = new I2cDeviceSynchImpl(color, colorAddr, false);
        colorReader.engage();
        //sensorColor = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        //sensorTouch = hwMap.get(TouchSensor.class, "sensor_touch");
    }

    //Function: set mode run using encoder
    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        driveBackLeft.setMode(runMode);
        driveBackRight.setMode(runMode);
        driveFrontLeft.setMode(runMode);
        driveFrontRight.setMode(runMode);
    }

    //Function: set power to all the motors
    public void setPowerAllDriveMotors(double speed) {
        driveBackLeft.setPower(speed);
        driveBackRight.setPower(speed);
        driveFrontLeft.setPower(speed);
        driveFrontRight.setPower(speed);
    }

    //Function: set power to all the motors
    public void setPowerLeftDriveMotors(double speed) {
        driveBackLeft.setPower(speed);
        driveFrontLeft.setPower(speed);
    }

    //Function: set power to all the motors
    public void setPowerRightDriveMotors(double speed) {
        driveBackRight.setPower(speed);
        driveFrontRight.setPower(speed);
    }

    public void setPositionClaw(double setPositionUp, double setPositionDown) {
        clawUpRight.setPosition(setPositionUp);
        clawUpLeft.setPosition(setPositionUp);
        clawDownLeft.setPosition(setPositionDown);
        clawDownRight.setPosition(setPositionDown);

    }
}

