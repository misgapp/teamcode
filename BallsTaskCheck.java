package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto balls task", group="Apollo")
public class BallsTaskCheck extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        ballsTaskAndReadPhoto(true);
    }

    public void ballsTask(boolean isRed) {
        boolean colorDetected = false;
        boolean frontIsRed = false;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            if (robot.coloradoFront.red() > 45 && robot.coloradoFront.red() > robot.colorabiBack.red() && robot.coloradoFront.blue() < robot.colorabiBack.blue() ) {
                frontIsRed = true;
                colorDetected = true;
                break;
            }

            if (robot.coloradoFront.red() < robot.colorabiBack.red() && robot.coloradoFront.blue() > 30 && robot.coloradoFront.blue() > robot.colorabiBack.blue() ) {
                frontIsRed = false;
                colorDetected = true;
                break;
            }
        }

        if (!colorDetected) {
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                if (robot.coloradoFront.red() > 45 && robot.coloradoFront.red() > robot.colorabiBack.red()) {
                    frontIsRed = true;
                    colorDetected = true;
                    break;
                }

                if (robot.coloradoFront.blue() > 30 && robot.coloradoFront.blue() > robot.colorabiBack.blue()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorabiBack.red() > 45 && robot.coloradoFront.red() < robot.colorabiBack.red()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorabiBack.blue() > 30 && robot.coloradoFront.blue() < robot.colorabiBack.blue()) {
                    frontIsRed = true;
                    colorDetected = true;
                    break;
                }
            }
        }

        telemetry.addData("front is red ", frontIsRed);
        telemetry.addData("color detected ", colorDetected);
        telemetry.update();

        if (colorDetected) {
            if (isRed == frontIsRed) {
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going back ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                sleep(100000000);

            } else {
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going front ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                sleep(10000000);

            }
            telemetry.addData("column ", vuMark);
            telemetry.update();
        }

    }
}
