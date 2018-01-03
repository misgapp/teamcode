package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

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
    VuforiaTrackables relicTrackables;
    static final double DROP_RIGHT_BALL_POSITION = 1;
    static final double DROP_LEFT_BALL_POSITION = 0.5;
    public static final double DROP_POSITION_ARM_RIGHT_LEFT = 0.8;
    public static final double DROP_POSITION_ARM_UP_DOWN = 0.1;
    public static final double START_POSITION_ARM_UP_DOWN = 1;

    static final double START_POSITION = 0.05;

    double speed = 0.2;

    public void apolloInit() {
        robot.init(hardwareMap);
        initVuforia();
    }

    void apolloRun(boolean isRed, boolean isCorner) {
        robot.prepareForStart();
        setClaw();
        ballsTask(isRed);
        RelicRecoveryVuMark column = readPhoto();
        reportImage(column);
        moveToCryptoBox(isRed, isCorner, column);
        putCube();
    }


//    void readColor() {
//        final int LED_ON = 0;
//
//        robot.colorReader.write8(3, LED_ON);
//
//        while (opModeIsActive()) {
//            int color = robot.colorReader.read8(4);
//
//            telemetry.addData("ball color: ", color);
//            telemetry.update();
//            idle();
//        }
//    }

    public void setClaw() {
        robot.setPositionClaw(0.5, 0.5);
        robot.lift.setPower(-speed);
        sleep(1500);
        robot.lift.setPower(0);
    }

    // Balls task: Move the ball with the other color aside.
    public void ballsTask(boolean isRed) {
        robot.armRightLeft.setPosition(DROP_POSITION_ARM_RIGHT_LEFT);

        robot.armUpDown.setPosition(0.5);
        sleep(300);
        robot.armUpDown.setPosition(0.3);
        sleep(200);

        robot.armRightLeft.setPosition(0.5);

        robot.armUpDown.setPosition(DROP_POSITION_ARM_UP_DOWN);
        sleep(500);

        final int LED_ON = 0;
        final int LED_OFF = 1;
        final int COLOR_BLUE = 3;
        final int COLOR_RED = 10;

        //robot.colorReader.write8(3, LED_ON);

        int colorRed = 0;
        int colorBlue = 0;
        for (int i = 0; i < 100; i++) {
            colorRed = robot.colorabi.red();
            colorBlue = robot.colorabi.blue();
            if (colorRed != 0 && colorBlue != 0) {
                break;
            }
            for (int e = 0; e < 100; e++) {
                colorBlue = robot.colorado.red();
                colorRed = robot.colorado.blue();
                if (colorRed != 0 && colorBlue != 0) {
                    break;
                }
            }
        }

        int coloradoColorRed = 0;
        int coloradoColorBlue = 0;


        telemetry.addData("ball color: ", colorRed);
        telemetry.update();

        if (colorRed == COLOR_BLUE) {
            if (isRed) {
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            } else {
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            }
        } else if (colorRed == COLOR_RED) {
            if (isRed) {
                robot.armRightLeft.setPosition(DROP_LEFT_BALL_POSITION);
            } else {
                robot.armRightLeft.setPosition(DROP_RIGHT_BALL_POSITION);
            }
        }

        sleep(1000);

        robot.armRightLeft.setPosition(DROP_POSITION_ARM_RIGHT_LEFT);
        robot.armUpDown.setPosition(START_POSITION_ARM_UP_DOWN);

        sleep(1000);
    }

    // Read photo and return the column to put the cube in.
    public RelicRecoveryVuMark readPhoto() {
        relicTrackables.activate();
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
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

        final int TICK_TO_CRYPTO_BOX_CORNER = 4250;
        final int TICK_TO_CRYPTO_BOX_COLUMN_WALL = 500;

        if (isRed) {
            if (column == RelicRecoveryVuMark.LEFT) {
                column = RelicRecoveryVuMark.RIGHT;
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                column = RelicRecoveryVuMark.LEFT;
            }
        }

        //close servo claw
        //robot.setPositionClaw(up,down);

        if (isCorner) {
            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_CORNER * direction);
            } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 900) * direction);
            } else {
                driveStrait(speed, (TICK_TO_CRYPTO_BOX_CORNER + 1800) * direction);
            }

            turn(speed, false);
            driveStrait(speed, 1600);
        } else {
            driveStrait(speed, 3700 * direction);
            turn(speed, true);

            if (column == RelicRecoveryVuMark.LEFT) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL);
            } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 800);
            } else {
                driveStrait(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 1600);
            }

            //turn(speed, TURN_2_CRYPTO_BOX_WALL * direction, -1 * TURN_2_CRYPTO_BOX_WALL * direction);
            turn(speed, !isRed);
            driveStrait(speed, 1300);
        }
    }

    // Put the cube
    public void putCube() {
        robot.setPositionClaw(1, 1);
        driveStrait(speed, -500);
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    public void driveStrait(double speed, int tick) {
        encoderDrive(speed, tick, tick);
    }

    public void turn(double speed, boolean turn_left) {
        int ticks = 2100;
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

    void reportImage(RelicRecoveryVuMark column) {
        if (column == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("Image", "Center");
        } else if (column == RelicRecoveryVuMark.LEFT) {
            telemetry.addData("Image", "Left");
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("Image", "Right");
        } else {
            telemetry.addData("Image", "Unknown");
        }
        telemetry.update();
    }
}
