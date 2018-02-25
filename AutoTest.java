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

        telemetry.addData("Blue back ", robot.colorBack.blue());
        telemetry.addData("Red back ", robot.colorBack.red());
        telemetry.addData("Blue front", robot.colorFront.blue());
        telemetry.addData("Red front", robot.colorFront.red());
        telemetry.update();
        sleep(10000000);

        /*
        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.2, 4000, 0+90*(i-1));
                gyroTurn(0.2, 90*i);
            }
        }

        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.9, 4000, 0+90*(i-1));
                gyroTurn(0.9, 90*i);
            }
        }

        gyroTurn(0.2, -90);

        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.2, -4000, 0+90*(i-1));
                gyroTurn(0.2, -90*i);
            }
        }

        for (int i = 3; i<7; i++){
            robot.setPositionClaw(0.1*i, 0.1*i);
        }

*/
    }
}
