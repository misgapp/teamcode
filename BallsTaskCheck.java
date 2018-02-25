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
            if (robot.colorFront.red() > 45 && robot.colorFront.red() > robot.colorBack.red() && robot.colorFront.blue() < robot.colorBack.blue() ) {
                frontIsRed = true;
                colorDetected = true;
                break;
            }

            if (robot.colorFront.red() < robot.colorBack.red() && robot.colorFront.blue() > 30 && robot.colorFront.blue() > robot.colorBack.blue() ) {
                frontIsRed = false;
                colorDetected = true;
                break;
            }
        }

        if (!colorDetected) {
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                if (robot.colorFront.red() > 45 && robot.colorFront.red() > robot.colorBack.red()) {
                    frontIsRed = true;
                    colorDetected = true;
                    break;
                }

                if (robot.colorFront.blue() > 30 && robot.colorFront.blue() > robot.colorBack.blue()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorBack.red() > 45 && robot.colorFront.red() < robot.colorBack.red()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorBack.blue() > 30 && robot.colorFront.blue() < robot.colorBack.blue()) {
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
                telemetry.addData("Blue back ", robot.colorBack.blue());
                telemetry.addData("Red back ", robot.colorBack.red());
                telemetry.addData("Blue front", robot.colorFront.blue());
                telemetry.addData("Red front", robot.colorFront.red());
                telemetry.update();
                sleep(100000000);

            } else {
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going front ", colorDetected);
                telemetry.addData("Blue back ", robot.colorBack.blue());
                telemetry.addData("Red back ", robot.colorBack.red());
                telemetry.addData("Blue front", robot.colorFront.blue());
                telemetry.addData("Red front", robot.colorFront.red());
                telemetry.update();
                sleep(10000000);

            }
            telemetry.addData("column ", vuMark);
            telemetry.update();
        }

    }
}
