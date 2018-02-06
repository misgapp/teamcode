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

        moreCubs(true);
        /*
        for (int g = 0; g<4; g++){
            for (int i = 0; i<4; i++){
                driveStrait(0.2, 4000);
                turn(0.2, false);
            }
        }
        turn(0.2, true);
        for (int g = 0; g<4; g++){
            for (int i = 0; i<4; i++){
                driveStrait(0.2, -4000);
                turn(0.2, true);
            }
        }

*/
    }
}
