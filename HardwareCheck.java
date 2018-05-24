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


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.2, 1500, 0+90*(i-1));
                gyroTurn(0.2, 90*i);
            }
        }

        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.9, 1500, 0+90*(i-1));
                gyroTurn(0.9, 90*i);
            }
        }

        gyroTurn(0.2, -90);

        for (int g = 0; g<2; g++){
            for (int i = 1; i<5; i++){
                gyroDrive(0.2, -1500, 0+90*(i-1));
                gyroTurn(0.2, -90*i);
            }
        }

        for (int i = 3; i<7; i++){
            robot.setPositionClaw(0.1*i, 0.1*i);
        }
    }

}
