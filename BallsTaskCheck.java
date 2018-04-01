package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto balls task", group="Apollo")
public class BallsTaskCheck extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        while (opModeIsActive()){
            ballsTaskAndReadPhoto1(true);

        }

    }

    public BallColor getBallColor(double distance,
                                  int red,
                                  int blue){
        if (red >= 40) {
            if (blue >= 40) {
                return BallColor.UNKOWN;
            }
            return BallColor.RED;
        } else {
            if (blue >= 40) {
                return BallColor.BLUE;
            }
        }

        if (distance < 6) {
            return BallColor.UNKOWN;
        }

        if (red > 30 && blue < 20) {
            return BallColor.RED;
        }

        if (red < 20 && blue > 30) {
            return BallColor.BLUE;
        }

        return BallColor.UNKOWN;
    }

    public BallColor getFrontBallColor(double distanceFront,
                                       int redFront,
                                       int blueFront,
                                       double distanceBack,
                                       int redBack,
                                       int blueBack) {
        if (distanceBack > 10){
            if (distanceFront > 10){
                return BallColor.UNKOWN;
            } else {
                return getBallColor(distanceFront, redFront, blueFront);
            }
        } else if (distanceFront > 10){
            BallColor backColor = getBallColor(distanceBack, redBack, blueBack);
            BallColor frontColor = BallColor.UNKOWN;

            if (backColor == BallColor.RED) {
                frontColor = BallColor.BLUE;
            } else if (backColor == BallColor.BLUE) {
                frontColor = BallColor.RED;
            }
            return frontColor;
        } else {
            BallColor frontColor = getBallColor(distanceFront, redFront, blueFront);
            BallColor backColor = getBallColor(distanceBack, redBack, blueBack);

            if (frontColor == BallColor.UNKOWN) {
                return backColor == BallColor.RED ? BallColor.BLUE : BallColor.RED;
            }
            if (backColor == BallColor.UNKOWN) {
                return frontColor;
            }
            if (frontColor != backColor) {
                return frontColor;
            }
        }
        return BallColor.UNKOWN;
    }


    // Balls task: Move the ball with the other color aside.
    public void ballsTaskAndReadPhoto1(boolean isRed) {
        robot.armRightLeft.setPosition(0.38);
        robot.armUpDown.setPosition(0.5);
        readPhotoWhileWait(200);
        robot.armUpDown.setPosition(0.7);
        readPhotoWhileWait(250);
        robot.armUpDown.setPosition(0.74);
        readPhotoWhileWait(350);
        robot.armUpDown.setPosition(0.78);
        readPhotoWhileWait(200);

        boolean colorDetected = false;
        boolean frontIsRed = false;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            BallColor frontColor = getFrontBallColor(
                    robot.coloradoDistanceFront.getDistance(DistanceUnit.CM),
                    robot.coloradoFront.red(),
                    robot.coloradoFront.blue(),
                    robot.colorabiDistanceBack.getDistance(DistanceUnit.CM),
                    robot.colorabiBack.red(),
                    robot.colorabiBack.blue());
            if (frontColor != BallColor.UNKOWN) {
                colorDetected = true;
                frontIsRed = (frontColor == BallColor.RED);
                break;
            }
        }

        telemetry.addData("front is red ", frontIsRed);
        telemetry.addData("color detected ", colorDetected);
        telemetry.update();

        if (colorDetected) {
            if (isRed == frontIsRed) {
                robot.armRightLeft.setPosition(0.00);
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going back ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                readPhotoWhileWait(300);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.3);
                readPhotoWhileWait(100);
                robot.armRightLeft.setPosition(0.4);
            } else {
                robot.armRightLeft.setPosition(0.8);
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going front ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                readPhotoWhileWait(300);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.5);
                readPhotoWhileWait(100);
                robot.armRightLeft.setPosition(0.4);

            }
        }

        sleep(2500);
    }

}