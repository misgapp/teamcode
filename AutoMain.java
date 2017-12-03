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

        int direction = isRed ? -1 : 1;

        final int TICK_TO_CRYPTO_BOX_CORNER = 2500;
        final int TICK_TO_CRYPTO_BOX_COLUMN_WALL = 1700;
        final int TURN_1_CRYPTO_BOX_WALL = 1200;
        final int TURN_2_CRYPTO_BOX_WALL = 1200;
        final int TURN_CRYPTO_BOX_CORNER = 1200;

        if (isRed){
            if (column == RelicRecoveryVuMark.LEFT) {
                column = RelicRecoveryVuMark.RIGHT;
            } else  if (column == RelicRecoveryVuMark.RIGHT){
                column = RelicRecoveryVuMark.LEFT;
            }
        }

        if (isCorner) {
            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_CORNER * direction);
            } else if (column == RelicRecoveryVuMark.CENTER) {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 1000) * direction);
            } else {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 1000) * direction);
            }

            //turn(speed, TURN_CRYPTO_BOX_CORNER * direction, -1 * TURN_CRYPTO_BOX_CORNER * direction);
            turn(speed, isRed);
        } else {
            driveStrait(speed, 300 * direction);
            //turn(speed, -1 * TURN_1_CRYPTO_BOX_WALL * direction, TURN_1_CRYPTO_BOX_WALL * direction);
            turn(speed, !isRed);

            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL * direction);
            } else if (column == RelicRecoveryVuMark.CENTER) {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_COLUMN_WALL + 1000) * direction);
            } else {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_COLUMN_WALL + 1000) * direction);
            }

            //turn(speed, TURN_2_CRYPTO_BOX_WALL * direction, -1 * TURN_2_CRYPTO_BOX_WALL * direction);
            turn(speed, isRed);
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

    public void turn(double speed, boolean turn_left) {
        int ticks = 2000;
        encoderDrive(speed, turn_left ? ticks : -ticks, turn_left ? -ticks : ticks);
    }

    // function drive encoder
    // speed - power level between 0 and 1. Is always positive.
    // tickRight - ticks of right side to drive. If positive driving towards cube claw.
    //   If negative drives to the other direction.
    public void encoderDrive(double speed, int tickRight, int tickLeft) {
        robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        speed = Math.abs(speed);
        double leftSpeed = tickLeft > 0 ? -speed : speed;
        double rightSpeed = tickRight > 0 ? -speed : speed;

        newLeftTarget = robot.driveBackLeft.getCurrentPosition() + tickLeft;
        newRightTarget = robot.driveBackRight.getCurrentPosition() + tickRight;

        robot.setPowerLeftDriveMotors(leftSpeed);
        robot.setPowerRightDriveMotors(rightSpeed);

        while (opModeIsActive()) {
            if (tickLeft > 0) {
                if (robot.driveBackLeft.getCurrentPosition() >= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() >= newLeftTarget) {
                    telemetry.addData("break", "1");
                    break;
                }
            } else {
                if (robot.driveBackLeft.getCurrentPosition() <= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() <= newLeftTarget) {
                    telemetry.addData("break", "2");
                    break;
                }
            }
            if (tickRight > 0) {
                if (robot.driveBackRight.getCurrentPosition() >= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() >= newRightTarget) {
                    telemetry.addData("break", "3");
                    break;
                }
            } else {
                if (robot.driveBackRight.getCurrentPosition() <= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() <= newRightTarget) {
                    telemetry.addData("break", "4");
                    break;
                }
            }
            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveFrontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.setPowerAllDriveMotors(0);


        /*
        telemetry.addData("newLeftTarget", "%d", newLeftTarget);
        telemetry.addData("newRightTarget", "%d", newRightTarget);
        telemetry.addData("tickLeft", "%d", tickLeft);
        telemetry.addData("tickRight", "%d", tickRight);

        telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
        telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
        telemetry.addData("tick right", "%d", robot.driveFrontRight.getCurrentPosition());
        telemetry.update();
       */

    }


    /*
    // function drive encoder
    // speed - power level between 0 and 1. Is always positive.
    // tickRight - ticks of right side to drive. If positive driving towards cube claw.
    //   If negative drives to the other direction.
    public void encoderDrive(double speed, int tickRight, int tickLeft) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        speed = Math.abs(speed);
        double leftSpeed = tickLeft > 0 ? -speed : speed;
        double rightSpeed = tickRight > 0 ? -speed : speed;

        tickLeft = tickLeft < 0 ? -tickLeft : tickLeft;
        tickRight = tickRight < 0 ? -tickRight : tickRight;

        //newLeftTarget = tickLeft > 0 ? robot.driveBackLeft.getCurrentPosition() + tickLeft :
                //robot.driveBackLeft.getCurrentPosition() - tickLeft;
        //newRightTarget = tickLeft > 0 ? robot.driveBackRight.getCurrentPosition() + tickLeft :
                //robot.driveBackRight.getCurrentPosition() - tickLeft;

        //newLeftTarget = robot.driveBackLeft.getCurrentPosition() + tickLeft;
        //newRightTarget = robot.driveBackRight.getCurrentPosition() + tickRight;

        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setPowerLeftDriveMotors(leftSpeed);
        robot.setPowerRightDriveMotors(rightSpeed);

        while (opModeIsActive()) {
            if (tickLeft < 0) {
                if (robot.driveBackLeft.getCurrentPosition() >= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() >= newLeftTarget) {
                    break;
                }
            } else {
                if (robot.driveBackLeft.getCurrentPosition() <= newLeftTarget ||
                        robot.driveFrontLeft.getCurrentPosition() <= newLeftTarget) {
                    break;
                }
            }
            if (tickRight < 0) {
                if (robot.driveBackRight.getCurrentPosition() >= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() >= newRightTarget) {
                    break;
                }
            } else {
                if (robot.driveBackRight.getCurrentPosition() <= newRightTarget ||
                        robot.driveFrontRight.getCurrentPosition() <= newRightTarget) {
                    break;
                }
            }
            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }


        robot.setPowerAllDriveMotors(0);
    }

    */
}
