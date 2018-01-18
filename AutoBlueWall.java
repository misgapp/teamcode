package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto blue wall", group="Apollo")
public class AutoBlueWall extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        //apolloRun(false, false);

        ballsTask(true);
    }
}
