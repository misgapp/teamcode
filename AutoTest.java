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

        //gyroTurn(speed, 180);

        encoderLift1(1,-1000);




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
