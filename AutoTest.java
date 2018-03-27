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

        while (opModeIsActive()) {
            telemetry.addData("numbers", "%f,%d,%d,%f,%d,%d",
                    robot.coloradoDistanceFront.getDistance(DistanceUnit.CM),
                    robot.coloradoFront.red(),
                    robot.coloradoFront.blue(),
                    robot.colorabiDistanceBack.getDistance(DistanceUnit.CM),
                    robot.colorabiBack.red(),
                    robot.colorabiBack.blue());
            telemetry.update();
        }


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
