package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto test", group="Apollo")
public class AutoTest extends AutoMain {
    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();



        robot.halfCloseClaws();
        //P_DRIVE_COEFF = 0.17;
        robot.setPositionWheel(robot.GRAB_POSITION);
        speed = 0.5;
        int ticks = gyroDrive(speed, 2400, 90, true, false);
        if ((robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15
                && robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15)
                || robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15){
            robot.closeClaws();
            sleep(100);
            robot.lift.setPower(1);
            sleep(400);
            robot.lift.setPower(0);
            gyroDrive(speed, -ticks, 90);

        } else if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15){
            robot.closeClawsDown();
            sleep(100);
            robot.lift.setPower(1);
            sleep(500);
            robot.lift.setPower(0);
            ticks += gyroDrive(speed, -800, 90);
            robot.spinner.setPower(0.8);
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            while (opModeIsActive() && robot.touchSpinnerDown.getState() && runtime.seconds() < 2){
                idle();
            }
            robot.spinner.setPower(0);
            goUpSpin = false;
            ticks += gyroDrive(speed, 1400, 90, true, false);
            robot.closeClaws();
            robot.lift.setPower(1);
            sleep(400);
            robot.lift.setPower(0);
            gyroDrive(speed, -(ticks ), 90);
        }
        //gyroDrive(speed, -1700, 90);
        robot.setPositionWheel(robot.STOP_POSITION);
        //P_DRIVE_COEFF = 0.08;

/*
        while (opModeIsActive()) {
            for (int g = 0; g < 4; g++) {
                for (int i = 1; i < 5; i++) {
                    gyroDrive(0.5, 2000, 0 + 90 * (i - 1));
                    gyroTurn(0.9, 90 * i);
                }
            }
        }
        */
    }
}
