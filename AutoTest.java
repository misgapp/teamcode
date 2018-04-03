package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto test", group="Apollo")
public class AutoTest extends AutoMain {
    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        moreCubsWall(RelicRecoveryVuMark.CENTER);

        //gyroTurn(speed, 180);

        // encoderLift1(1,-1000);






            /*
            telemetry.addData("numbers", "%f\n%d\n,%d\n,%f\n,%d\n,%d\n",
                    robot.coloradoDistanceFront.getDistance(DistanceUnit.CM),
                    robot.coloradoFront.red(),
                    robot.coloradoFront.blue(),
                    robot.colorabiDistanceBack.getDistance(DistanceUnit.CM),
                    robot.colorabiBack.red(),
                    robot.colorabiBack.blue());
            telemetry.update();

*/

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

    public void moreCubsWall(RelicRecoveryVuMark column){
        boolean grabCube = false;

        robot.openClaws();
        //startLift(1100); ///////////////////////////////////////////////////////////////////
        gyroDrive(speed, -300, 0);

        // Default values for right.
        double angleToCubes = -45;
        double angleToBox = 0;
        int ticksToMeetingPoint = 2000;
        int ticksToBox = 1700;

        if (column == RelicRecoveryVuMark.CENTER) {
            angleToCubes = -35;
            ticksToMeetingPoint = 1600;
            angleToBox = -7;
            ticksToBox = 1700;
        } else if (column == RelicRecoveryVuMark.LEFT) {
            angleToCubes = -25;
            ticksToMeetingPoint = 1300;
            angleToBox = -30;
            ticksToBox = 1700;
        }
        gyroTurn(speed, angleToCubes);
        gyroDrive(speed, -ticksToMeetingPoint, angleToCubes);
        double grabAngle = 160;
        gyroTurn(speed, grabAngle);

        robot.halfCloseClaws();

        robot.setPositionWheel(robot.GRAB_POSITION);
        int ticks = gyroDrive(speed, 3000, grabAngle, true, true);

        grabCube = (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15 ||
                robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15);

        robot.closeClaws();
        sleep(100);
        encoderLift(1, -2000);
        gyroDrive(speed, -300, grabAngle);
        ticks -= 300;
        goUpSpin = false; // Spin the claws to drop third cube.
        gyroDrive(speed, -ticks, grabAngle);

        gyroTurn(speed, angleToBox);
        encoderLift(1, 1000);
        gyroDrive(speed, ticksToBox, angleToBox);

        // Drop cube.
        encoderLift(1, 1000);
        robot.setPositionWheel(robot.DROP_POSITION);
        sleep(650);

        driveStrait(speed, 600);

        driveStrait(speed, -450);
        robot.setPositionWheel(robot.STOP_POSITION);
        robot.openClaws();
    }



    public void moreCubs(boolean isCorner, RelicRecoveryVuMark column){
        boolean grabCube = false;
        if (isCorner){
            robot.openClaws();
            startLift(1100);
            gyroDrive(speed, -1650, -90);
            //encoderLift(1 , 1100);
            gyroTurn(speed, 90);
            //gyroHold(speed, 90, 0.7);
            robot.halfCloseClaws();
            //P_DRIVE_COEFF = 0.17;
            robot.setPositionWheel(robot.GRAB_POSITION);
            int ticks = gyroDrive(speed, 2600, 90, true, true);

            if ((robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15
                    && robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15)
                    || robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15){
                // If got two cubes or only upper cube.
                grabCube = true;
                robot.closeClaws();
                sleep(100);
                //startLift(-2000);
                encoderLift(1, -2000);
                gyroDrive(speed, -300, 90);
                ticks -= 300;
                goUpSpin = false; // Spin the claws to drop third cube.
                gyroDrive(speed, -ticks +400, 90);

            } else if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15){
                // If got only lower go back, spin and try again.
                grabCube = true;
                if (overallTimer.seconds() <= 16) {
                    robot.closeClawsDown();
                    sleep(100);
                    startLift(-2000);
                    //encoderLift(1, -2000);
                    gyroDrive(speed, -400, 90);
                    ticks -= 400;
                    robot.spinner.setPower(0.8);
                    ElapsedTime runtime = new ElapsedTime();
                    runtime.reset();
                    while (opModeIsActive() && robot.touchSpinnerDown.getState() && runtime.seconds() < 2) {
                        handleLift();
                        idle();
                    }
                    robot.spinner.setPower(0);
                    goUpSpin = false;
                    encoderLift(1, 2000);
                    ticks += gyroDrive(speed, 1400, 90, true, false);
                    robot.closeClaws();
                    encoderLift(1, -2000);
                    gyroDrive(speed, -300, 90);
                    ticks -= 300;
                    goUpSpin = true; // Spin the claws to drop third cube.
                    gyroDrive(speed, -ticks + 400, 90);
                } else {
                    robot.closeClaws();
                    sleep(100);
                    encoderLift(1, -400);
                    gyroDrive(speed, -ticks + 400, 90);
                }
            }

            if (grabCube){
                //gyroDrive(speed, -1700, 90);
                robot.closeClaws();
                robot.setPositionWheel(robot.STOP_POSITION);
                //startLift(-3000);
                double angle = (column == RelicRecoveryVuMark.RIGHT) ? -50 : -130;
                gyroTurn(speed, angle);
                //gyroHold(speed, 180, 1);
                //sleep(100);

                encoderLift(1, 400);
                gyroDrive(speed, 1000, angle);
                gyroTurn(speed, -90);
                encoderLift(1, 1000);
                gyroDrive(speed, 1100, -90);
                robot.setPositionWheel(robot.DROP_POSITION);
                sleep(650);
                //gyroDrive(speed, -150, -90);
                driveStrait(speed, 600);
                //robot.openClaws();
                driveStrait(speed, -450);
                robot.setPositionWheel(robot.STOP_POSITION);
                robot.openClaws();
            } else {
                gyroDrive(speed, -ticks -500, 90);
            }
        }
    }


    public void encoderLift1(double speed, int tick) {
        encoderLiftTimer = new ElapsedTime();

        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget = 0;

        int timeS = (Math.abs(tick)/1000) * 1; // Second per 1000 ticks

        speed = Math.abs(speed);
        speed = tick > 0 ? -speed : speed;

        newTarget = robot.liftLeft.getCurrentPosition() + tick;

        encoderLiftTimer.reset();
        robot.setPowerLifts(speed);

        while (opModeIsActive() && encoderLiftTimer.seconds() < timeS){
            if (tick > 0) {
                if (robot.liftLeft.getCurrentPosition() >= newTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.liftLeft.getCurrentPosition() <= newTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }

            telemetry.addData("tick", "%d", robot.liftLeft.getCurrentPosition());
            telemetry.update();
            idle();

        }

        robot.setPowerLifts(0);
    }
}
