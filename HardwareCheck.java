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

        driveStrait(0.3, 3250);
        driveStrait(0.3, -3250);
        turn(0.3, true);
        turn(0.3, false);
    }

}
