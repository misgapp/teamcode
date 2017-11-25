package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Carmel on 10/11/2017.
 * <p>
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class AutoMain extends LinearOpMode {

    HardwareApollo robot = new HardwareApollo();
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    static final double DROP_RIGHT_BALL_POSITION = 0.05;
    static final double DROP_LEFT_BALL_POSITION = 0.05;
    static final double CHECK_COLOR_POSITION = 0.05;
    static final double START_POSITION = 0.05;

    double speed = 0.2;

    public void apolloInit() {
        robot.init(hardwareMap);
        initVuforia();
    }

    void apolloRun(boolean isRed, boolean isCorner) {
        //ballsTask(isRed);
        RelicRecoveryVuMark column = readPhoto();
        moveToCryptoBox(isRed, isCorner, column);
        putCube();
    }

    /* Balls task: Move the ball with the other color aside.
    public void ballsTask(boolean isRed) {
        robot.armUpDown.setPosition(CHECK_COLOR_POSITION);

        telemetry.addData("ball color blue", robot.sensorColor.blue());
        telemetry.addData("ball color red", robot.sensorColor.red());
        telemetry.update();

        if (robot.sensorColor.blue() > 20) {
            if (isRed) {
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            } else {
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            }
        } else if (robot.sensorColor.red() > 10) {
            if (isRed) {
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            } else {
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            }
        }

        robot.armUpDown.setPosition(START_POSITION);
    }

    */

    // Read photo and return the column to put the cube in.
    public RelicRecoveryVuMark readPhoto() {
        for (int i = 0; i < 3; i++) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark;
            }
        }
        return RelicRecoveryVuMark.UNKNOWN;
    }

    // Move to crypto box
    public void moveToCryptoBox(boolean isRed, boolean isCorner, RelicRecoveryVuMark column) {
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int direction = isRed ? 1 : -1;

        final int TICK_TO_CRYPTO_BOX_CORNER = 300;
        final int TICK_TO_CRYPTO_BOX_COLUMN_WALL = 20;
        final int TURN_1_CRYPTO_BOX_WALL = 30;
        final int TURN_2_CRYPTO_BOX_WALL = 30;
        final int TURN_CRYPTO_BOX_CORNER = 30;

        if (isCorner) {
            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_CORNER * direction);
            } else if (column == RelicRecoveryVuMark.CENTER) {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 10) * direction);
            } else {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 40) * direction);
            }

            turn(speed, TURN_CRYPTO_BOX_CORNER * direction, -1 * TURN_CRYPTO_BOX_CORNER * direction);
        } else {
            driveStrait(speed, 300 * direction);
            turn(speed, -1 * TURN_1_CRYPTO_BOX_WALL * direction, TURN_1_CRYPTO_BOX_WALL * direction);

            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL * direction);
            } else if (column == RelicRecoveryVuMark.CENTER) {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_COLUMN_WALL + 10) * direction);
            } else {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_COLUMN_WALL + 40) * direction);
            }

            turn(speed, TURN_2_CRYPTO_BOX_WALL * direction, -1 * TURN_2_CRYPTO_BOX_WALL * direction);
        }
    }

    // Put the cube
    public void putCube() {
        robot.setPositionClaw(0.1, 0.9);
        driveStrait(speed, 50);
    }

    //init vuforia
    public void initVuforia() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AV/gkAT/////AAAAGWiIBaw6x0OTo+0xvZByyZw0Kd0bZituG7LwKLe13Ongw2Q6ffE9IAidA2GblBNIywUNa49sxB6NP+RXBL0D0GQ+FhyiPmnsgAm3nDNFcTfS3MN7QmRBbMD/OsBkGa4kuA7QWw/zOCumYjoLabqE6ZsBda/GWUhBxtfxNn9PeXaHOCsFZjnGC8AlRAt2jPcAVGZC+ZY/L6LFWTfnwETi5188NS25SHNz92pOj9YWT2vx8NCtsHj+OJot+QG2i6qQO9mRY83jpYrfqcIcE8w7ExCcNwirdU1lHT/mnlcqlqfLz9gwCd+qxE5ocY8PcSpNOjQmu2/Y4PA7TZOK4Qn9R6fn9hsrJNTSU3n2zgG99ZbB";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    public void driveStrait(double speed, int tick) {
        encoderDrive(speed, tick, tick);
    }

    public void turn(double speed, int tickRight, int tickLeft) {
        encoderDrive(speed, tickRight, tickLeft);
    }

    //function drive encoder
    public void encoderDrive(double speed, int tickRight, int tickLeft) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        newLeftTarget = robot.driveBackLeft.getCurrentPosition() + tickLeft;
        newRightTarget = robot.driveBackRight.getCurrentPosition() + tickRight;
        robot.driveBackLeft.setTargetPosition(newLeftTarget);
        robot.driveBackRight.setTargetPosition(newRightTarget);

        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPowerAllDriveMotors(speed);

        while (opModeIsActive() &&
                (robot.driveBackLeft.isBusy() || robot.driveBackRight.isBusy()
                        /*||
                        robot.driveFrontLeft.isBusy() || robot.driveFrontRight.isBusy()*/)) {
            idle();
        }

        robot.setPowerAllDriveMotors(0);
    }
}
