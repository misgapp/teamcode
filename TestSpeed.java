package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: test speed", group="Apollo")
public class TestSpeed extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        driveStrait(0.05, 3000);
        driveStrait(0.1, 3000);
        driveStrait(0.2, 3000);
        driveStrait(1, 3000);



    }
}
