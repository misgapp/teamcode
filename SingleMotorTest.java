package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Carmel on 10/11/2017.
 *
 * Check robot hardware.
 */
@Autonomous(name="Apollo: speed check", group="Apollo")
public class SingleMotorTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "m");

        motor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.2);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("current", "%d", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(1);
        runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("current", "%d", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(-1);
        runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("current", "%d", motor.getCurrentPosition());
            telemetry.update();
        }


    }

}


