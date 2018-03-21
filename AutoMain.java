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
    private double P_DRIVE_COEFF = 0.03; //For option to change in grab more cubes
    private static final int RED_THRESHOLD = 50;
    private static final int BLUE_THRESHOLD = 50;
    double speed = 0.8;

    //Init function, hardwareMap,vuforia and gyro calibration
    public void apolloInit() {
        robot.init(hardwareMap);
        robot.setPositionClaw(robot.START_POSITION_CLAW_UP, robot.START_POSITION_CLAW_DOWN);
        robot.armUpDown.setPosition(0.25);
        robot.armRightLeft.setPosition(0.8);
        robot.relicClaw.setPosition(robot.START_POSITION_CLAW);
        robot.setPositionWheel(robot.STOP_POSITION);
        initVuforia();
    }

    //The main function of the autonomous
    void apolloRun(boolean isRed, boolean isCorner) {
        robot.prepareForStart();
        setClaw();
        ballsTaskAndReadPhoto(isRed);
        RelicRecoveryVuMark column = vuMark;
        reportImage(column);
        moveToCryptoBox(isRed, isCorner, column);
        putCube();
        moreCubs(isCorner);
    }

    //Set the claws & lift to right position, grab cube
    public void setClaw() {
        robot.setPositionClaw(0.75, 0.25);
        robot.setPositionWheel(robot.GRAB_POSITION);
        readPhotoWhileWait(100);
        robot.setPositionWheel(robot.STOP_POSITION);
    }

    // Balls task: Move the ball with the other color aside.
    public void ballsTaskAndReadPhoto(boolean isRed) {
        robot.armRightLeft.setPosition(0.38);
        robot.armUpDown.setPosition(0.5);
        readPhotoWhileWait(400);
        robot.armUpDown.setPosition(0.78);
        readPhotoWhileWait(350);
        robot.armUpDown.setPosition(0.81);
        readPhotoWhileWait(450);
        robot.armUpDown.setPosition(0.83);
        readPhotoWhileWait(400);

        boolean colorDetected = false;
        boolean frontIsRed = false;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            if (robot.coloradoFront.red() > 30 && robot.coloradoFront.red() > robot.colorabiBack.red() && robot.coloradoFront.blue() < robot.colorabiBack.blue() ) {
                frontIsRed = true;
                colorDetected = true;
                break;
            }

            if (robot.coloradoFront.red() < robot.colorabiBack.red() && robot.coloradoFront.blue() > 30 && robot.coloradoFront.blue() > robot.colorabiBack.blue() ) {
                frontIsRed = false;
                colorDetected = true;
                break;
            }
        }

        if (!colorDetected) {
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                if (robot.coloradoFront.red() > 30 && robot.coloradoFront.red() > robot.colorabiBack.red()) {
                    frontIsRed = true;
                    colorDetected = true;
                    break;
                }

                if (robot.coloradoFront.blue() > 30 && robot.coloradoFront.blue() > robot.colorabiBack.blue()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorabiBack.red() > 30 && robot.coloradoFront.red() < robot.colorabiBack.red()) {
                    frontIsRed = false;
                    colorDetected = true;
                    break;
                }

                if (robot.colorabiBack.blue() > 30 && robot.coloradoFront.blue() < robot.colorabiBack.blue()) {
                    frontIsRed = true;
                    colorDetected = true;
                    break;
                }
            }
        }

        telemetry.addData("front is red ", frontIsRed);
        telemetry.addData("color detected ", colorDetected);
        telemetry.update();

        if (colorDetected) {
            if (isRed == frontIsRed) {
                robot.armRightLeft.setPosition(0.00);
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going back ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                robot.lift.setPower(0.95);
                readPhotoWhileWait(400);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.3);
                readPhotoWhileWait(700);
                robot.armRightLeft.setPosition(0.4);
                robot.lift.setPower(0);

            } else {
                robot.armRightLeft.setPosition(0.8);
                telemetry.addData("front is red ", frontIsRed);
                telemetry.addData("color detected ", colorDetected);
                telemetry.addData("going front ", colorDetected);
                telemetry.addData("Blue back ", robot.colorabiBack.blue());
                telemetry.addData("Red back ", robot.colorabiBack.red());
                telemetry.addData("Blue front", robot.coloradoFront.blue());
                telemetry.addData("Red front", robot.coloradoFront.red());
                telemetry.update();
                robot.lift.setPower(0.95);
                readPhotoWhileWait(400);
                robot.armUpDown.setPosition(0.6);
                robot.armRightLeft.setPosition(0.5);
                readPhotoWhileWait(700);
                robot.armRightLeft.setPosition(0.4);
                robot.lift.setPower(0);

            }
        } else {
            robot.lift.setPower(1);
            sleep(500);
            robot.lift.setPower(0);
        }
        robot.armUpDown.setPosition(0.25);
        telemetry.addData("column ", vuMark);
        telemetry.update();
        readPhotoWhileWait(450);
    }

    // Read photo while wait instead sleep
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

    // Move to crypto box
    public void moveToCryptoBox(boolean isRed, boolean isCorner, RelicRecoveryVuMark column) {
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int direction = isRed ? 1 : -1;
        int columnTicks = isRed ? 0 : 525;
        int gyroDegrees = isRed ? 0 : 180;
        int blue = isRed ? -200 : 0;

        final int TICK_TO_CRYPTO_BOX_CORNER = 2150;
        final int TICK_TO_CRYPTO_BOX_COLUMN_WALL = 450;

        if (isRed) {
            if (column == RelicRecoveryVuMark.LEFT) {
                column = RelicRecoveryVuMark.RIGHT;
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                column = RelicRecoveryVuMark.LEFT;
            }
        }

        if (isCorner) {
            if (column == RelicRecoveryVuMark.LEFT ) {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER) * direction, 0);
            } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER + 700) * direction, 0);
            } else {
                gyroDrive(speed, (columnTicks + TICK_TO_CRYPTO_BOX_CORNER + 1300) * direction, 0);
            }

            gyroTurn(speed, -90);
            gyroHold(speed, -90, 1);
            gyroDrive(speed, 900, -90); //Go closer to crypto
        } else {
            gyroDrive(speed, (2100 + columnTicks) * direction, 0);

            gyroTurn(speed, 90);
            gyroHold(speed, 90, 1);
            if (column == RelicRecoveryVuMark.LEFT || column == RelicRecoveryVuMark.UNKNOWN) {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL, 90);
            } else if (column == RelicRecoveryVuMark.CENTER ) {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 700, 90);
            } else {
                gyroDrive(speed, TICK_TO_CRYPTO_BOX_COLUMN_WALL + 1300, 90);
            }

            //turn(speed, TURN_2_CRYPTO_BOX_WALL * direction, -1 * TURN_2_CRYPTO_BOX_WALL * direction);
            gyroTurn(speed, 0+gyroDegrees);
            gyroHold(speed, 0+gyroDegrees, 1);
            driveStrait(speed, 400+blue); //Go closer to crypto
        }
    }

    // Put the cube in crypto box
    public void putCube() {
        robot.setPositionWheel(robot.DROP_POSITION);
        sleep(800);
        robot.setPositionWheel(robot.STOP_POSITION);
        driveStrait(speed, 400);
        if (robot.sensorDistanceCrypto.getDistance(DistanceUnit.CM) < 6){
            robot.setPositionClaw(0.7, 0.3);
            robot.setPositionWheel(robot.DROP_POSITION);
            driveStrait(speed, -400);
            driveStrait(speed, 400);
            sleep(1000);
            robot.setPositionWheel(robot.STOP_POSITION);
        }
        robot.setPositionClaw(1, 1);
        driveStrait(speed, -700);
    }

    // Put more cube in crypto box
    public void moreCubs(boolean isCorner){
        if (isCorner){
            robot.setPositionClaw(0.6, 0.4);
            gyroDrive(speed, -950, -90);
            robot.lift.setPower(-0.15);
            gyroTurn(speed, 90);
            robot.lift.setPower(0.0);
            gyroHold(speed, 90, 0.7);
            robot.setPositionClaw(0.7, 0.3);
            //P_DRIVE_COEFF = 0.17;
            robot.setPositionWheel(robot.GRAB_POSITION);
            gyroDrive(speed, 2400, 90);
            gyroDrive(speed, -1700, 90);
            //P_DRIVE_COEFF = 0.08;
            robot.lift.setPower(0.9);
            gyroTurn(speed, -90);
            gyroHold(speed, -90, 1);
            robot.lift.setPower(0.0);
            gyroDrive(speed, 2050, -90);
            robot.setPositionWheel(robot.DROP_POSITION);
            sleep(650);
            robot.setPositionClaw(0.3, 0.7);
            gyroDrive(speed, -400, -90);
            robot.setPositionClaw(0.5, 0.5);

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

    //Function drive encoder for turns
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
            telemetry.addData("tick left", "%d", robot.driveBackLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveBackRight.getCurrentPosition());
            telemetry.addData("tick left", "%d", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("tick right", "%d", robot.driveFrontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.setPowerAllDriveMotors(0);
    }

    //Telemetry to the column
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

    public void gyroDrive ( double speed,
                            double distanceTick,
                            double angle) {
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
            //moveCounts = (int)(distance * COUNTS_PER_INCH);
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

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.driveBackLeft.getCurrentPosition(),
                        robot.driveBackRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.setPowerAllDriveMotors(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
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
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /*
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.setPowerAllDriveMotors(0);
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

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

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
}