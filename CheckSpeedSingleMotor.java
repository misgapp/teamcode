package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Carmel on 10/11/2017.
 *
 * Check robot hardware.
 */
@Autonomous(name="Apollo: speed check", group="Apollo")
public class CheckSpeedSingleMotor extends AutoMain {

    HardwareApollo robot = new HardwareApollo();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive1(1, 1000);
        sleep(500);
        encoderDrive1(1, -1000);
        sleep(500);
        encoderDrive1(1, 1000);
        sleep(500);
        encoderDrive1(1, -1000);
        sleep(500);


    }

    public void encoderDrive1(double speed, int tick) {

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget = 0;

        speed = Math.abs(speed);
        speed = tick > 0 ? -speed : speed;

        newTarget = robot.lift.getCurrentPosition() + tick;

        robot.lift.setPower(speed);

        while (opModeIsActive()){
            if (tick > 0) {
                if (robot.lift.getCurrentPosition() >= newTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.lift.getCurrentPosition() <= newTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }

            telemetry.addData("tick", "%d", robot.lift.getCurrentPosition());
            telemetry.update();
            idle();

        }

        robot.lift.setPower(0);
    }

    public void encoderDrive(double speed, int tickRight, int tickLeft) {
        robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        speed = Math.abs(speed);
        double leftSpeed = tickLeft > 0 ? speed : -speed;
        double rightSpeed = tickRight > 0 ? speed : -speed;

        newLeftTarget = robot.driveBackLeft.getCurrentPosition() + tickLeft;
        newRightTarget = robot.driveBackRight.getCurrentPosition() + tickRight;

        robot.setPowerLeftDriveMotors(leftSpeed);
        robot.setPowerRightDriveMotors(rightSpeed);

        while (opModeIsActive()) {
            if (tickLeft > 0) {
                if (robot.driveBackLeft.getCurrentPosition() >= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() >= newLeftTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.driveBackLeft.getCurrentPosition() <= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() <= newLeftTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }
            if (tickRight > 0) {
                if (robot.driveBackRight.getCurrentPosition() >= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() >= newRightTarget) {
                    telemetry.addData("break", "3");
                    break;
                }
            } else {
                if (robot.driveBackRight.getCurrentPosition() <= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() <= newRightTarget) {
                    telemetry.addData("break", "4");
                    break;
                }
            }
            telemetry.addData("tick bleft", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick bright", "%d", robot.driveBackRight.getCurrentPosition());
            telemetry.addData("tick fleft", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("tick fright", "%d", robot.driveFrontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.setPowerAllDriveMotors(0);


        /*
        telemetry.addData("newLeftTarget", "%d", newLeftTarget);
        telemetry.addData("newRightTarget", "%d", newRightTarget);
        telemetry.addData("tickLeft", "%d", tickLeft);
        telemetry.addData("tickRight", "%d", tickRight);

        telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
        telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveFrontRight.getCurrentPosition());
        telemetry.update();
       */

    }

}


