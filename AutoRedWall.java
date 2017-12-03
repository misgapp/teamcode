package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto red wall", group="Apollo")
public class AutoRedWall extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        driveStrait(0.2, 5000);
        turn(0.2, false);
        driveStrait(0.2, 1000);

        //apolloRun(true, false);
    }
}
