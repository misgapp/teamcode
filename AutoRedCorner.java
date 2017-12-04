package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto red corner", group="Apollo")
public class AutoRedCorner extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
       apolloInit();

        waitForStart();

        apolloRun(true, true);

    }
}
