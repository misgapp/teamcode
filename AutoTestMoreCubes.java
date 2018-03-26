package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto test", group="Apollo")
public class AutoTestMoreCubes extends AutoMain {
    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        //gyroHold(speed, 90, 0.7);
        robot.halfCloseClaws();
        //P_DRIVE_COEFF = 0.17;
        robot.setPositionWheel(robot.GRAB_POSITION);
        speed = 0.5;
        int ticks = gyroDrive(speed, 2400, 0, true, false);
        if ((robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15
                && robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15)
                || robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15){
            robot.closeClaws();
            sleep(100);
            startLift(-1000);
            //encoderLift(1, -1000);
            gyroDrive(speed, -ticks, 0);

        } else if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15){
            robot.closeClawsDown();
            sleep(100);
            startLift(-2000);
            //encoderLift(1, -2000);
            ticks += gyroDrive(speed, -800, 0);
            robot.spinner.setPower(0.8);
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            while (opModeIsActive() && robot.touchSpinnerDown.getState() && runtime.seconds() < 2){
                handleLift();
                idle();
            }
            robot.spinner.setPower(0);
            goUpSpin = false;
            encoderLift(1, 1700);
            //encoderLift(1, 1700);
            ticks += gyroDrive(speed, 1400, 0, true, false);
            robot.closeClaws();
            encoderLift(1, -1800);
            //encoderLift(1, -1800);
            gyroDrive(speed, -(ticks), 0);
        }
        //gyroDrive(speed, -1700, 90);
        robot.closeClaws();
        robot.setPositionWheel(robot.STOP_POSITION);
        startLift(-4200);
        gyroTurn(speed, 180);
        //gyroHold(speed, 180, 1);
        sleep(100);
        goUpSpin = false; // Spin the claws to drop third cube.
        gyroDrive(speed, 1950, 180);
        robot.setPositionWheel(robot.DROP_POSITION);
        sleep(650);
        gyroDrive(speed, -150, 180);
        gyroDrive(speed, 150, 180);
        //robot.openClaws();
        gyroDrive(speed, -400, 180);
        robot.openClaws();

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
