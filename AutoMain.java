package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    static final double HEADING_THRESHOLD = 2 ;
    static final double P_TURN_COEFF = 0.02;
    private double P_DRIVE_COEFF = 0.03; //For option to change in grab more cubes.
    public boolean goUpSpin = true;
    double speed = 0.8;
    int liftTargetTicks = 0;
    boolean liftStopIfMoreThanTicks = false;
    boolean liftEnabled = false;
    ElapsedTime liftTimer;
    ElapsedTime overallTimer;
    int liftTimeoutSeconds;
    ElapsedTime encoderLiftTimer;
    ElapsedTime liftRelic;

    // The options to color ball front.
    public enum BallColor {
        UNKOWN,
        RED,
        BLUE
    }

    //Init function, hardwareMap,vuforia and gyro calibration
    public void apolloInit() {
        robot.init(hardwareMap);
        liftRelic = new ElapsedTime();
        liftRelic.reset();
        robot.relicLift.setPower(1);
        robot.setPositionClaw(0.1, 0.9);
        robot.armUpDown.setPosition(0.15);
        robot.armRightLeft.setPosition(0.8);
        robot.relicClaw.setPosition(robot.START_POSITION_CLAW);
        robot.setPositionWheel(robot.STOP_POSITION);
        initVuforia();
        while (liftRelic.seconds() < 1){
            idle();
        }
        telemetry.addData("I'm ","ready");
        telemetry.update();
        robot.relicLift.setPower(0.15);
    }

    //The main function of the autonomous
    void apolloRun(boolean isRed, boolean isCorner) {
        robot.relicLift.setPower(0); // Stop the relic lift motor.
        overallTimer = new ElapsedTime(); // Create new elapsed time for cube.
        overallTimer.reset();
        robot.prepareForStart(); // Prepare gyro for start.
        setClaw(); // Set claw to start position with cube.
        ballsTaskAndReadPhoto(isRed); // Ball task and read photo function.
        RelicRecoveryVuMark column = vuMark;
        moveToCryptoBox(isRed, isCorner, column); // Move to cryptoBox function.
        putCube(isCorner); // Put the cube in cryptoBox.
        if (isCorner) {
            moreCubsCorner(column); // More cubes corner function.
        } else {
            moreCubsWall(column, isRed); // More cubes wall function.
        }
    }

    //Set the claws & lift to right position, grab cube.
    public void setClaw() {
        robot.closeClaws();
        robot.setPositionWheel(robot.GRAB_POSITION);
        readPhotoWhileWait(100);
        robot.setPositionWheel(robot.STOP_POSITION);
    }

    // Detect the color of the ball according to red, blue, distance value.
    public BallColor getBallColor(double distance,
                                  int red,
                                  int blue){
        if (red >= 40) {
            if (blue >= 40) {
                return BallColor.UNKOWN;
            }
            return BallColor.RED;
        } else {
            if (blue >= 40) {
                return BallColor.BLUE;
            }
        }

        if (distance < 6) {
            return BallColor.UNKOWN;
        }

        if (red > 30 && blue < 20) {
            return BallColor.RED;
        }

        if (red < 20 && blue > 30) {
            return BallColor.BLUE;
        }

        return BallColor.UNKOWN;
    }

    // Detect front color using distance sensor and get ball color function.
    public BallColor getFrontBallColor(double distanceFront,
                                       int redFront,
                                       int blueFront,
                                       double distanceBack,
                                       int redBack,
                                       int blueBack) {
        if (distanceBack > 10){
            if (distanceFront > 10){
                return BallColor.UNKOWN;
            } else {
                return getBallColor(distanceFront, redFront, blueFront);
            }
        } else if (distanceFront > 10){
            BallColor backColor = getBallColor(distanceBack, redBack, blueBack);
            BallColor frontColor = BallColor.UNKOWN;

            if (backColor == BallColor.RED) {
                frontColor = BallColor.BLUE;
            } else if (backColor == BallColor.BLUE) {
                frontColor = BallColor.RED;
            }
            return frontColor;
        } else {
            BallColor frontColor = getBallColor(distanceFront, redFront, blueFront);
            BallColor backColor = getBallColor(distanceBack, redBack, blueBack);

            if (frontColor == BallColor.UNKOWN) {
                return backColor == BallColor.RED ? BallColor.BLUE : BallColor.RED;
            }
            if (backColor == BallColor.UNKOWN) {
                return frontColor;
            }
            if (frontColor != backColor) {
                return frontColor;
            }
        }
        return BallColor.UNKOWN;
    }

    // Balls task: Move the ball with the other color aside.
    public void ballsTaskAndReadPhoto(boolean isRed) {
        // Set the servos to drop position.
        robot.armRightLeft.setPosition(0.38);
        robot.armUpDown.setPosition(0.5);
        readPhotoWhileWait(200);
        robot.armUpDown.setPosition(0.7);
        readPhotoWhileWait(250);
        robot.armUpDown.setPosition(0.74);
        readPhotoWhileWait(350);
        robot.armUpDown.setPosition(0.78);
        readPhotoWhileWait(200);

        int colorDetected = 0;
        boolean frontIsRed = false;

        // Detect color ball three times to sure the color is correct.
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            BallColor frontColor = getFrontBallColor(
                    robot.coloradoDistanceFront.getDistance(DistanceUnit.CM),
                    robot.coloradoFront.red(),
                    robot.coloradoFront.blue(),
                    robot.colorabiDistanceBack.getDistance(DistanceUnit.CM),
                    robot.colorabiBack.red(),
                    robot.colorabiBack.blue());
            if (frontColor != BallColor.UNKOWN) {
                boolean newDetectionFrontIsRed = (frontColor == BallColor.RED);
                if (newDetectionFrontIsRed != frontIsRed) {
                    colorDetected = 1;
                } else {
                    colorDetected++;
                    sleep(20); // Sleep 20 milli seconds between the readings.
                }
                frontIsRed = newDetectionFrontIsRed;
                if (colorDetected >= 3) {
                    break;
                }
            }
        }

        // Drop ball according the variables and lift the lift.
        if (colorDetected >= 3) {
            if (isRed == frontIsRed) {
                robot.armRightLeft.setPosition(0.00);
                encoderLift(1 , -1300);
                readPhotoWhileWait(300);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.3);
                readPhotoWhileWait(100);
                robot.armRightLeft.setPosition(0.4);
            } else {
                robot.armRightLeft.setPosition(0.8);
                encoderLift(1 , -1300);
                readPhotoWhileWait(300);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.5);
                readPhotoWhileWait(100);
                robot.armRightLeft.setPosition(0.4);
            }
        } else {
            encoderLift(1 , -1300);
        }
        robot.armUpDown.setPosition(0.15);

        // If didn't detect the photo try again.
        if (vuMark == RelicRecoveryVuMark.UNKNOWN){
            readPhotoWhileWait(450);
        }

        // Stop trying to read the photo.
        relicTrackables.deactivate();
    }

    // Read photo while wait instead sleep.
    public void readPhotoWhileWait(int time) {
        relicTrackables.activate();
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < time) {
            if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
        }
    }

    // Move to crypto box.
    public void moveToCryptoBox(boolean isRed, boolean isCorner, RelicRecoveryVuMark column) {
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Variable to correct the different between red and blue.
        int direction = isRed ? 1 : -1;
        int columnTicks = isRed ? 0 : 525;
        int gyroDegrees = isRed ? 0 : 180;
        int blue = isRed ? 0 : -100;
        int blueCorner = isRed ? 0 : 100;

        final int TICK_TO_CRYPTO_BOX_CORNER = 2150; // Ticks to the closer column in crypto corner.
        final int TICK_TO_CRYPTO_BOX_COLUMN_WALL = 450; // Ticks to the closer column in crypto wall.

        // That is the different between red and blue in column.
        if (isRed) {
            if (column == RelicRecoveryVuMark.LEFT) {
                column = RelicRecoveryVuMark.RIGHT;
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                column = RelicRecoveryVuMark.LEFT;
            }
        }

        // Drive to correct column according the variable.
        if (isCorner) {
            if (column == RelicRecoveryVuMark.LEFT ) {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER) * direction, 0);
            } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER + 700) * direction, 0);
            } else {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER + 1300) * direction, 0);
            }

            gyroTurn(speed, -90);
            gyroDrive(speed, blueCorner, -90);
        } else {
            gyroDrive(speed, (2100 + columnTicks) * direction, 0);

            gyroTurn(speed, 90);
            if (column == RelicRecoveryVuMark.LEFT) {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL, 90);
            } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 650 + blue, 90);
            } else {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 1350 + blue, 90);
            }

            gyroTurn(speed, 0+gyroDegrees);
        }
    }

    // Put the cube in crypto box.
    public void putCube(boolean isCorner) {
        driveStrait(speed, 500);
        encoderLift(1, 300);
        robot.setPositionWheel(robot.DROP_POSITION);
        sleep(800);
        driveStrait(speed, 500);
        robot.setPositionWheel(robot.STOP_POSITION);
        // Try put the cube another time if sense it with the distance sensor.
        if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 6){
            robot.closeClaws();
            robot.setPositionWheel(robot.DROP_POSITION);
            driveStrait(speed, -400);
            driveStrait(speed, 400);
            sleep(1000);
            robot.setPositionWheel(robot.STOP_POSITION);
        }
        robot.openClaws();
        if (!isCorner) {
            driveStrait(speed, -200);
        }
    }

    // Put more two cube in crypto box corner.
    public void moreCubsCorner(RelicRecoveryVuMark column){
        boolean grabCube = false;
        robot.openClaws();
        startLift(1100);
        gyroDrive(speed, -1650, -90);
        gyroTurn(speed, 90);
        robot.halfCloseClaws();
        robot.setPositionWheel(robot.GRAB_POSITION);
        int ticks = gyroDrive(speed, 2600, 90, true, true);

        if ((robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15 &&
                robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15) ||
                robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15) {
            // If got two cubes or only upper cube.
            grabCube = true;
            robot.closeClaws();
            sleep(100);
            encoderLift(1, -2000);
            gyroDrive(speed, -300, 90);
            ticks -= 300;
            goUpSpin = false; // Spin the claws to drop third cube.
            gyroDrive(speed, -ticks + 400, 90);

        } else if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15) {
            // If got only lower go back, spin and try again.
            grabCube = true;
            boolean enableDoubleTake = false;
            if (enableDoubleTake && overallTimer.seconds() <= 16) {
                robot.closeClawsDown();
                sleep(100);
                startLift(-2000);
                gyroDrive(speed, -400, 90);
                ticks -= 400;
                robot.spinner.setPower(0.8);
                ElapsedTime runtime = new ElapsedTime();
                runtime.reset();
                while (opModeIsActive() && robot.touchSpinnerDown.getState() && runtime.seconds() < 2) {
                    handleLift(); // Stop lift.
                    idle();
                }
                robot.spinner.setPower(0);
                goUpSpin = false;
                encoderLift(1, 2000);
                ticks += gyroDrive(speed, 1400, 90, true, false);
                robot.closeClaws();
                encoderLift(1, -2000);
                gyroDrive(speed, -300, 90);
                ticks -= 300;
                goUpSpin = true; // Spin the claws to drop third cube.
                gyroDrive(speed, -ticks + 400, 90);
            } else {
                robot.closeClaws();
                sleep(100);
                encoderLift(1, -400);
                gyroDrive(speed, -ticks + 400, 90);
            }
        }

        if (grabCube) {
            robot.closeClaws();
            robot.setPositionWheel(robot.STOP_POSITION);
            double angle = (column == RelicRecoveryVuMark.RIGHT) ? -50 : -130;
            gyroTurn(speed, angle);
            encoderLift(1, 400);
            gyroDrive(speed, 1000, angle);
            gyroTurn(speed, -90);
            encoderLift(1, 1000);
            gyroDrive(speed, 1100, -90);
            robot.setPositionWheel(robot.DROP_POSITION);
            sleep(650);
            driveStrait(speed, 600);
            driveStrait(speed, -450);
            robot.setPositionWheel(robot.STOP_POSITION);
            robot.openClaws();
        } else {
            gyroDrive(speed, -ticks - 500, 90);
        }
    }

    // Put more two cube in crypto box wall.
    public void moreCubsWall(RelicRecoveryVuMark column, boolean isRed){
        robot.openClaws();
        encoderLift(1, 1000);
        gyroDrive(speed, -300, 0);

        // Default values for right for red or left for blue.
        double angleToCubes = -45;
        double angleToBox = 0;
        int ticksToMeetingPoint = 2000;
        int ticksToBox = 1700;

        if (column == RelicRecoveryVuMark.CENTER) {
            angleToCubes = -35;
            ticksToMeetingPoint = 1600;
            angleToBox = -7;
            ticksToBox = 1700;
        } else if (column == RelicRecoveryVuMark.LEFT && isRed ||
                column == RelicRecoveryVuMark.RIGHT && !isRed) {
            angleToCubes = -25;
            ticksToMeetingPoint = 1300;
            angleToBox = -30;
            ticksToBox = 1700;
        }

        if (!isRed) {
            angleToCubes += 180;
            angleToCubes *= -1;
            angleToBox += 180;
            angleToBox *= -1;
        }

        gyroTurn(speed, angleToCubes);
        gyroDrive(speed, -ticksToMeetingPoint, angleToCubes);
        double grabAngle = isRed ? 160 : 20;
        gyroTurn(speed, grabAngle);

        robot.halfCloseClaws();

        robot.setPositionWheel(robot.GRAB_POSITION);
        int ticks = gyroDrive(speed, 3300, grabAngle, true, true);

        robot.closeClaws();
        sleep(100);
        encoderLift(1, -2000);
        gyroDrive(speed, -300, grabAngle);
        ticks -= 300;
        goUpSpin = false; // Spin the claws to drop third cube.
        gyroDrive(speed, -ticks, grabAngle);

        if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 15 ||
                robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 15) {
            gyroTurn(speed, angleToBox);
            encoderLift(1, 1000);
            gyroDrive(speed, ticksToBox, angleToBox);

            // Drop cube.
            encoderLift(1, 1000);
            robot.setPositionWheel(robot.DROP_POSITION);
            sleep(650);

            driveStrait(speed, 600);

            driveStrait(speed, -350);
            robot.setPositionWheel(robot.STOP_POSITION);
            robot.openClaws();
        } else {
            gyroDrive(speed, -1000, grabAngle);
        }
    }


    //Init vuforia
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

    //Function drive encoder for drives strait
    public void driveStrait(double speed, int tick) {
        encoderDrive(speed, tick, tick);
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
        double leftSpeed = tickLeft > 0 ? speed : -speed;
        double rightSpeed = tickRight > 0 ? speed : -speed;

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
            idle();

            handleLift(); // Stop lift.
        }
        robot.setPowerAllDriveMotors(0);
    }

    public int gyroDrive(double speed,
                          double distanceTick,
                          double angle) {
        return gyroDrive(speed, distanceTick, angle, false, false);
    }

    /*
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distanceTick   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */

    public int gyroDrive(double speed,
                          double distanceTick,
                          double angle,
                          boolean stopIfSenseUpCube,
                          boolean stopIfSenseDownCube) {
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distanceTick);
            newBackLeftTarget = robot.driveBackLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.driveBackRight.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.driveFrontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.driveFrontRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.driveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.driveBackRight.setTargetPosition(newBackRightTarget);
            robot.driveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.driveFrontRight.setTargetPosition(newFrontRightTarget);

            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.setPowerAllDriveMotors(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.driveBackLeft.isBusy() &&
                    robot.driveBackRight.isBusy() &&
                    robot.driveFrontLeft.isBusy() &&
                    robot.driveFrontRight.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceTick < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setPowerLeftDriveMotors(leftSpeed);
                robot.setPowerRightDriveMotors(rightSpeed);

                // Spin the spinner while drive.
                if (goUpSpin){
                    if (robot.touchSpinnerUp.getState()){
                        robot.spinner.setPower(-0.8);
                    } else{
                        robot.spinner.setPower(0);
                    }
                } else {
                    if (robot.touchSpinnerDown.getState()){
                        robot.spinner.setPower(0.8);
                    } else if (!robot.touchSpinnerDown.getState()) {
                        robot.spinner.setPower(0);
                    }
                }

                // Stop drive if grab lower cube, for the more cube part.
                if (stopIfSenseDownCube){
                    if (robot.sensorDistanceDown.getDistance(DistanceUnit.CM) < 6){
                        break;
                    }
                }

                // Stop drive if grab upper cube, for the more cube part.
                if (stopIfSenseUpCube){
                    if (robot.sensorDistanceUp.getDistance(DistanceUnit.CM) < 6){
                        break;
                    }
                }

                handleLift(); // Stop lift.
            }

            // Stop all motion;
            robot.setPowerAllDriveMotors(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return robot.driveFrontRight.getCurrentPosition();

        }
        return 0;
    }

    /*
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {
        robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            handleLift(); // Stop lift.
        }
    }

    /*
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerLeftDriveMotors(leftSpeed);
        robot.setPowerRightDriveMotors(rightSpeed);

        return onTarget;
    }

    /*
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //Encoder lift function.
    public void encoderLift(double speed, int tick) {
        encoderLiftTimer = new ElapsedTime();

        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget = 0;

        int timeS = (Math.abs(tick)/1000) * 1; // Second per 1000 ticks

        speed = Math.abs(speed);
        speed = tick > 0 ? -speed : speed;

        newTarget = robot.liftLeft.getCurrentPosition() + tick;

        encoderLiftTimer.reset();
        robot.setPowerLifts(speed);

        while (opModeIsActive() && encoderLiftTimer.seconds() < timeS){
            if (tick > 0) {
                if (robot.liftLeft.getCurrentPosition() >= newTarget) {
                    break;
                }
            } else {
                if (robot.liftLeft.getCurrentPosition() <= newTarget) {
                    break;
                }
            }

        }

        robot.setPowerLifts(0);
    }

    // Stop the lift if grt to target, while drive gyro function.
    public void handleLift() {
        if (!liftEnabled){
            return;
        }

        int ticks = robot.liftRight.getCurrentPosition();
        if ((liftStopIfMoreThanTicks && ticks >= liftTargetTicks) ||
                (!liftStopIfMoreThanTicks && ticks <= liftTargetTicks)||
                liftTimer.seconds() >= liftTimeoutSeconds) {
            robot.setPowerLifts(0);
            liftEnabled = false;
        }
    }

    // Set all the variables for the lift get the right position while do other things.
    public void startLift(int ticks) {
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTargetTicks = ticks;
        liftStopIfMoreThanTicks = ticks > 0;
        liftTimeoutSeconds = (Math.abs(ticks)/1000) * 1; // Second per 1000 ticks
        liftTimer = new ElapsedTime();
        liftTimer.reset();
        liftEnabled = true;
        robot.setPowerLifts(ticks > 0 ? -1:1);
    }
}