package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by Apollo9662 on 11/29/2017.
 */

@Autonomous(name="Apollo: Auto test RunWithEncoder1", group="Apollo")
public class AutoTestRunWithEncoderAuto extends AutoMain {
    HardwareApollo robot = new HardwareApollo();

    public void runOpMode() {
        robot.init(hardwareMap);

        driveStrait(0.3, 1500);
        /*
        robot.driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int ticksLeft = robot.driveFrontLeft.getCurrentPosition();
        int ticksRight = robot.driveFrontRight.getCurrentPosition();

        waitForStart();

        robot.setPowerAllDriveMotors(-0.2);

        while (opModeIsActive() &&
                robot.driveFrontLeft.getCurrentPosition() < ticksLeft + 2500 &&
                robot.driveFrontRight.getCurrentPosition() < ticksLeft + 2500 &&
                robot.driveBackRight.getCurrentPosition() < ticksLeft + 2500 &&
                robot.driveBackLeft.getCurrentPosition() < ticksRight + 2500){

            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveFrontLeft.getCurrentPosition());
            //telemetry.addData("drive direction forward", "%.2f", driveDirectionForward);
            telemetry.update();
            idle();
        }

        robot.setPowerAllDriveMotors(0);

        telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
        telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveFrontLeft.getCurrentPosition());

        */


    }
}
