package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Carmel on 10/11/2017.
 *
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class AutoMain extends LinearOpMode {

    public enum Column {
        LEFT, CENTER, RIGHT
    }

    HardwareApollo robot = new HardwareApollo();

    void apolloRun(boolean isRed, boolean isCorner){
        ballsTask(isRed);
        Column column = readPhoto();
        moveToCryptoBox(isRed, isCorner);
        putCube (column);
        park();
    }

    // Balls task: Move the ball with the other color aside.
    private void ballsTask(boolean isRed){
        // TODO(): implement.
    }

    // Read photo and return the column to put the cube in.
    private Column readPhoto(){
        // TODO(): implement.
        return Column.RIGHT; // Place holder.
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isRed, boolean isCorner){
        // TODO(): implement.
    }

    // Put the cube
    private void putCube (Column column){
        // TODO(): implement.
    }

    // Park the robot
    private void park (){
        // TODO(): implement.
    }


}
