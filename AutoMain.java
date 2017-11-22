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
 *
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
    static final int TICK_TO_CRYPTO_BOX = 300;
    int newLeftTarget = 0;
    int newRightTarget = 0;
    double speed = 0.2;

    public void apolloInit(){
        robot.init(hardwareMap);
        initVuforia();
    }

    void apolloRun(boolean isRed, boolean isCorner){
        ballsTask(isRed);
        RelicRecoveryVuMark column = readPhoto();
        moveToCryptoBox(isRed, isCorner);
        putCube(column);
        park();
    }

    // Balls task: Move the ball with the other color aside.
    private void ballsTask(boolean isRed){
        robot.armUpDown.setPosition(CHECK_COLOR_POSITION);

        telemetry.addData("ball color blue", robot.sensorColor.blue());
        telemetry.addData("ball color red", robot.sensorColor.red());

        if (robot.sensorColor.blue()>20){
            if (isRed){
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            }else {
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            }
        }else if (robot.sensorColor.red()>10){
            if (isRed){
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            }else{
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            }
        }

        robot.armUpDown.setPosition(START_POSITION);
    }

    // Read photo and return the column to put the cube in.
    private RelicRecoveryVuMark readPhoto(){
        for (int i=0; i<3; i++){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark;
            }
        }
        return RelicRecoveryVuMark.UNKNOWN;
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isRed, boolean isCorner){

        robot.driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isCorner){
            newLeftTarget = robot.driveBackLeft.getCurrentPosition() + TICK_TO_CRYPTO_BOX;
            newRightTarget = robot.driveBackRight.getCurrentPosition() + TICK_TO_CRYPTO_BOX;
            robot.driveBackLeft.setTargetPosition(newLeftTarget);
            robot.driveBackRight.setTargetPosition(newRightTarget);

            robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.driveBackLeft.setPower(speed);
            robot.driveBackRight.setPower(speed);
            robot.driveFrontLeft.setPower(speed);
            robot.driveFrontRight.setPower(speed);

            while (opModeIsActive() &&
                    (robot.driveBackLeft.isBusy() || robot.driveBackRight.isBusy()||
                            robot.driveFrontLeft.isBusy() || robot.driveFrontRight.isBusy())){
                idle();
            }

            robot.driveBackLeft.setPower(0);
            robot.driveBackRight.setPower(0);
            robot.driveFrontLeft.setPower(0);
            robot.driveFrontRight.setPower(0);
        }else {
            newLeftTarget = robot.driveBackLeft.getCurrentPosition() + TICK_TO_CRYPTO_BOX;
            newRightTarget = robot.driveBackRight.getCurrentPosition() + TICK_TO_CRYPTO_BOX;
            robot.driveBackLeft.setTargetPosition(newLeftTarget);
            robot.driveBackRight.setTargetPosition(newRightTarget);

            robot.driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    // Put the cube
    private void putCube (RelicRecoveryVuMark column){
        // TODO(): implement.
    }

    // Park the robot
    private void park (){
        // TODO(): implement.
    }

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
}
