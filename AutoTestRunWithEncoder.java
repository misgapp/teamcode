package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Apollo9662 on 11/29/2017.
 */

@Autonomous(name="Apollo: Auto test RunWithEncoder", group="Apollo")
public class AutoTestRunWithEncoder extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo();

    public void runOpMode() {
        robot.init(hardwareMap);

        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticksLeft = robot.driveBackLeft.getCurrentPosition();
        int ticksRight = robot.driveBackRight.getCurrentPosition();

        waitForStart();

        robot.setPowerAllDriveMotors(-0.2);

        while (opModeIsActive() &&
                (robot.driveBackLeft.getCurrentPosition() > ticksLeft + 2500
                   ||
                    robot.driveBackLeft.getCurrentPosition() > ticksRight + 2500)){
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
