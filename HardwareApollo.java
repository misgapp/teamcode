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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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
    public Servo wheelDownLeft = null;
    public Servo wheelDownRight = null;
    public Servo wheelUpLeft = null;
    public Servo wheelUpRight = null;
    public TouchSensor sensorTouch = null;
    //public ColorSensor sensorColor = null;
    public I2cAddr colorAddr = I2cAddr.create8bit(0x3c);
    public I2cDevice color = null;
    public I2cDeviceSynch colorReader = null;
    BNO055IMU imu;

    public TouchSensor sensor_button = null;

    public static final double START_POSITION_CLAW_UP = 1;
    public static final double START_POSITION_CLAW_DOWN = 1;
    public static final double START_POSITION_ARM_UP_DOWN = 1;
    public static final double START_POSITION_ARM_RIGHT_LEFT = 0.8;
    public static final double START_POSITION_RELIC_CLAW = 0.5;
    public static final double START_POSITION_RELIC_ARM = 0.5;
    public static final double STOP_POSITION= 0.5;
    public static final double DROP_POSITION= 1;
    public static final double GRAB_POSITION= 0;

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

        driveBackLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
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
        wheelDownLeft = hwMap.get(Servo.class, "wdl");
        wheelDownRight = hwMap.get(Servo.class, "wdr");
        wheelUpLeft = hwMap.get(Servo.class, "wul");
        wheelUpRight = hwMap.get(Servo.class, "wur");

        setPositionClaw(START_POSITION_CLAW_UP, START_POSITION_CLAW_DOWN);
        //armUpDown.setPosition(START_POSITION_ARM_UP_DOWN);
        //armRightLeft.setPosition(START_POSITION_ARM_RIGHT_LEFT);
        //relicArm.setPosition(START_POSITION_RELIC_ARM);
        //relicClaw.setPosition(START_POSITION_RELIC_CLAW);
        setPositionWheel(STOP_POSITION);

        //sensorColor = hwMap.get(ColorSensor.class, "sensor_color");
        color = hwMap.i2cDevice.get("sc");
        colorReader = new I2cDeviceSynchImpl(color, colorAddr, false);
        colorReader.engage();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

    //Function: set position to all the claws
    public void setPositionClaw(double setPositionUp, double setPositionDown) {
        clawUpRight.setPosition(setPositionUp);
        clawUpLeft.setPosition(1-setPositionUp);
        clawDownLeft.setPosition(1-setPositionDown);
        clawDownRight.setPosition(setPositionDown);
    }

    //Function: set position to all the wheel
    public void setPositionWheel(double setPosition) {
        wheelUpRight.setPosition(setPosition);
        wheelUpLeft.setPosition(setPosition);
        wheelDownLeft.setPosition(setPosition);
        wheelDownRight.setPosition(setPosition);
    }

    public void preaperForStart(){
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

}

