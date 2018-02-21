package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
}
