package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Carmel on 10/11/2017.
 */

@Autonomous(name="Apollo: Auto test", group="Apollo")
public class AutoTest extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
       apolloInit();

        waitForStart();

        driveStrait(0.2, 1000);

    }
}
