package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Apollo9662 on 11/29/2017.
 */

@Autonomous(name="Apollo: Auto test OR", group="Apollo")
public class AutoTestOR extends LinearOpMode {
    HardwareApollo robot   = new HardwareApollo();

    public void runOpMode(){
        robot.init(hardwareMap);

        robot.driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int newLeftTarget = 0;
        int newRightTarget = 0;

        newLeftTarget = robot.driveBackLeft.getCurrentPosition() + 1000;
        newRightTarget = robot.driveBackRight.getCurrentPosition() + 1000;
        robot.driveBackLeft.setTargetPosition(newLeftTarget);
        robot.driveBackRight.setTargetPosition(newRightTarget);

        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPowerAllDriveMotors(0.3);

        while (opModeIsActive() &&
                (robot.driveBackLeft.isBusy() || robot.driveBackRight.isBusy()
                        /*||
                        robot.driveFrontLeft.isBusy() || robot.driveFrontRight.isBusy()*/)) {
            idle();
            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            //telemetry.addData("drive direction forward", "%.2f", driveDirectionForward);
            telemetry.update();
        }

        robot.setPowerAllDriveMotors(0);

        telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());

    }

}
