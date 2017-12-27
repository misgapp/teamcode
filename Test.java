package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Apollo9662 on 12/27/2017.
 */
@Autonomous(name="Apollo: test speed", group="Apollo")
public class Test extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo();
    public void runOpMode() {
        robot.driveFrontRight.setPower(0);

        waitForStart();

        tick(4000, 0.1);
        tick(4000, 0.2);
        tick(4000, 1);
    }

    public void tick(int tick, double Speed) {
        int newRightTarget = 0;
        robot.driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newRightTarget = robot.driveBackRight.getCurrentPosition() + tick;

        robot.setPowerLeftDriveMotors(Speed);

        while (robot.driveBackLeft.getCurrentPosition() >= newRightTarget){
            idle();
        }
        robot.driveFrontRight.setPower(0);
    }
}
