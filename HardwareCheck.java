package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Carmel on 10/11/2017.
 *
 * Check robot hardware.
 */
@Autonomous(name="Apollo: Hardware check", group="Apollo")
public class HardwareCheck extends AutoMain{

    HardwareApollo robot = new HardwareApollo();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
    }

}
