package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by kyledufrene on 2/25/17.
 */


@Autonomous(name="Test Blue - Full Auto", group="SHS")




public class BlueAutoFull extends LinearOpMode {

    /* Declare OpMode members. */
    MarvHardware robot = new MarvHardware();   // Use a Pushbot's hardware
    ColorSensor colorSensorBeacon;
    ColorSensor colorSensorGroundFront;
    ColorSensor colorSensorGroundBack;
    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cGyro gyro2 = null; // Additional Gyro device

    static final double COUNTS_PER_MOTOR_REV = 1220;    // eg: TETRIX Motor Encoder = 1440, Andy Mark = 1220
    static final double DRIVE_GEAR_REDUCTION = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.05;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.1;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.01;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double ERRORCOEFF = 1;
    double frontLeft;
    double frontRight;

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro2 = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro2");
        colorSensorGroundBack = hardwareMap.colorSensor.get("color_sensor_ground_back");
        colorSensorGroundFront = hardwareMap.colorSensor.get("color_sensor_ground_front");
        colorSensorBeacon = hardwareMap.colorSensor.get("color_sensor_beacon");

        colorSensorBeacon.setI2cAddress(I2cAddr.create7bit(0x3E));
        colorSensorGroundBack.setI2cAddress(I2cAddr.create7bit(0x2E));
        colorSensorGroundFront.setI2cAddress(I2cAddr.create7bit(0x1E));
        gyro.setI2cAddress(I2cAddr.create7bit(0x18));
        gyro2.setI2cAddress(I2cAddr.create7bit(0x10));

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();
        gyro2.calibrate();

        //Send telemetry message to signify robot waiting;
        colorSensorGroundFront.enableLed(false);
        colorSensorGroundBack.enableLed(false);
        colorSensorBeacon.enableLed(false);
        gyro.resetZAxisIntegrator();
        gyro2.resetZAxisIntegrator();
        colorSensorGroundFront.enableLed(true);
        colorSensorGroundBack.enableLed(true);
        colorSensorBeacon.enableLed(false);

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating() && gyro2.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", getGyroAverage());
            //telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();
        gyro2.resetZAxisIntegrator();
        colorSensorGroundFront.enableLed(true);
        colorSensorGroundBack.enableLed(true);
        colorSensorBeacon.enableLed(false);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        drive(.3, -5);
        Wait(.25);
        turn(1, -30);
        Wait(.25);
        drive(.5, -73);
        Wait(.5);
        turn(1, -3);
        Wait(.25);
        setPowerLine(.25);
        Wait(1);
        beacon();
        Wait(5);
        setPowerLine(-.25);
        beacon2();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {
        int newLeftTarget;
        int newRightTarget;
        int newRightTargetBack;
        int newLeftTargetBack;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTargetBack = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetBack = robot.rightMotorBack.getCurrentPosition() + moveCounts;
            newLeftTarget = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotorBack.getCurrentPosition() + moveCounts;
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotorBack.setTargetPosition(newLeftTargetBack);
            robot.rightMotorBack.setTargetPosition(newRightTargetBack);
            robot.leftMotorFront.setTargetPosition(newLeftTarget);
            robot.rightMotorFront.setTargetPosition(newRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotorBack.setPower(leftSpeed);
                robot.rightMotorBack.setPower(rightSpeed);
                robot.leftMotorFront.setPower(leftSpeed);
                robot.rightMotorFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotorBack.setPower(0);
            robot.rightMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void gyroDrive2(double speed,
                           double distance,
                           double angle) {
        int newLeftTarget;
        int newRightTarget;
        int newRightTargetBack;
        int newLeftTargetBack;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTargetBack = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetBack = robot.rightMotorBack.getCurrentPosition() + moveCounts;
            newLeftTarget = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotorBack.getCurrentPosition() + moveCounts;
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotorBack.setTargetPosition(newLeftTargetBack);
            robot.rightMotorBack.setTargetPosition(newRightTargetBack);
            robot.leftMotorFront.setTargetPosition(newLeftTarget);
            robot.rightMotorFront.setTargetPosition(newRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotorBack.setPower(leftSpeed);
                robot.rightMotorBack.setPower(rightSpeed);
                robot.leftMotorFront.setPower(leftSpeed);
                robot.rightMotorFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotorBack.setPower(0);
            robot.rightMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void gyroDriveToWhiteLine(double speed, double distance, double angle,
                                     ColorSensor mColorSensor) {

        int newLeftTarget;
        int newRightTarget;
        int newRightTargetBack;
        int newLeftTargetBack;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        int white = mColorSensor.alpha();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTargetBack = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetBack = robot.rightMotorBack.getCurrentPosition() + moveCounts;
            newLeftTarget = robot.leftMotorBack.getCurrentPosition();
            newRightTarget = robot.rightMotorBack.getCurrentPosition();

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotorBack.setTargetPosition(newLeftTargetBack);
            robot.rightMotorBack.setTargetPosition(newRightTargetBack);
            robot.leftMotorFront.setTargetPosition(newLeftTarget);
            robot.rightMotorFront.setTargetPosition(newRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()) && white < 1) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotorBack.setPower(leftSpeed);
                robot.rightMotorBack.setPower(rightSpeed);
                robot.leftMotorFront.setPower(leftSpeed);
                robot.rightMotorFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotorBack.setPower(0);
            robot.rightMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotorBack.setPower(leftSpeed);
        robot.rightMotorBack.setPower(rightSpeed);
        robot.leftMotorFront.setPower(leftSpeed);
        robot.rightMotorFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroAverage();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //get average reading from both gyros
    private double getGyroAverage() {
        double g1 = gyro.getIntegratedZValue();
        double g2 = gyro2.getIntegratedZValue();

        return ((g1 + g2) / 2);
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void Wait(double time) {
        eTime.reset();
        while (eTime.time() < time && opModeIsActive()) {
        }
    }

    public void beacon() {
        int blue = colorSensorBeacon.blue();
        int red = colorSensorBeacon.red();

        if (blue >= red * 2 && blue > 4) {
            robot.beacon.setPosition(0);
            Wait(1);
            robot.beacon.setPosition(.75);
            Wait(.25);
            robot.beacon.setPosition(0);
            Wait(.25);
            robot.beacon.setPosition(1);
        } else if (red >= blue * 2 && red > 3) {
            setPower(.2,.2);
            while (colorSensorGroundBack.alpha() < .8 && opModeIsActive()) {

            }
            robot.beacon.setPosition(0);
            Wait(1);
            robot.beacon.setPosition(.75);
            Wait(.25);
            robot.beacon.setPosition(0);
            Wait(.25);
            robot.beacon.setPosition(1);

        }
    }
    public void beacon2() {
        int blue = colorSensorBeacon.blue();
        int red = colorSensorBeacon.red();

        if (blue >= red * 3 && blue > 6) {
            robot.beacon.setPosition(0);
            Wait(1);
            robot.beacon.setPosition(.75);
            Wait(.25);
            robot.beacon.setPosition(0);
            Wait(.25);
            robot.beacon.setPosition(1);
        } else if (red >= blue * 3 && red >4) {
            setPower(-.2,-.2);
            while (colorSensorGroundBack.alpha() < .8 && opModeIsActive()) {

            }
            robot.beacon.setPosition(0);
            Wait(1);
            robot.beacon.setPosition(.75);
            Wait(.25);
            robot.beacon.setPosition(0);
            Wait(.25);
            robot.beacon.setPosition(1);

        }
    }

    public void turn(double speed, double angle) {
        boolean onHeading = false;
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = error(angle);
        if (error <= ERRORCOEFF && error >= -ERRORCOEFF) {
            onHeading = true;
        }
        while (!onHeading && opModeIsActive()) {
            error = error(angle);
            boolean direction = false;
            if (error > 0) {
                direction = true;
            }
            if (direction && error > 30) {
                setPower(-.8, .8);
            } else if (!direction && error < -30) {
                setPower(.8, -.8);
            } else if (direction && error > 22) {
                setPower(-.7, .7);
            } else if (!direction && error < -22) {
                setPower(.7, -.7);
            } else if (direction && error > 10) {
                setPower(-.7, .7);
            } else if (!direction && error < -10) {
                setPower(.7, -.7);
            } else if (direction && error > 1) {
                setPower(-.65, .65);
            } else if (!direction && error < -1) {
                setPower(.65, -.65);
            }
            if (error <= ERRORCOEFF && error >= -ERRORCOEFF) {
                onHeading = true;
            }
        }
        setPower(0, 0);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double error(double targetAngle) {
        double error = targetAngle - getGyroAverage();
        return error;
    }

    public void setPower(double left, double right) {
        robot.leftMotorBack.setPower(left);
        robot.leftMotorFront.setPower(left);
        robot.rightMotorFront.setPower(right);
        robot.rightMotorBack.setPower(right);
    }

    public void drive(double speed, double distance) {
        int newLeftTarget;
        int newRightTarget;
        int newRightTargetBack;
        int newLeftTargetBack;
        int moveCounts;
        moveCounts = (int) (distance * COUNTS_PER_INCH);
        newLeftTargetBack = robot.leftMotorBack.getCurrentPosition() + moveCounts;
        newRightTargetBack = robot.rightMotorBack.getCurrentPosition() + moveCounts;
        newLeftTarget = robot.leftMotorBack.getCurrentPosition() + moveCounts;
        newRightTarget = robot.rightMotorBack.getCurrentPosition() + moveCounts;

        robot.leftMotorBack.setTargetPosition(newLeftTargetBack);
        robot.rightMotorBack.setTargetPosition(newRightTargetBack);
        robot.leftMotorFront.setTargetPosition(newLeftTarget);
        robot.rightMotorFront.setTargetPosition(newRightTarget);

        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.leftMotorBack.setPower(speed);
        robot.rightMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(speed);
        robot.rightMotorFront.setPower(speed);
        while (opModeIsActive() &&
                (robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy())) {
        }
// Stop all motion;
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorFront.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drivetoWhiteLineRear(double speed, double distance) {
        int newLeftTarget;
        int newRightTarget;
        int newRightTargetBack;
        int newLeftTargetBack;
        int moveCounts;
        int white = colorSensorGroundBack.alpha();
        moveCounts = (int) (distance * COUNTS_PER_INCH);
        newLeftTargetBack = robot.leftMotorBack.getCurrentPosition() + moveCounts;
        newRightTargetBack = robot.rightMotorBack.getCurrentPosition() + moveCounts;
        newLeftTarget = robot.leftMotorBack.getCurrentPosition() + moveCounts;
        newRightTarget = robot.rightMotorBack.getCurrentPosition() + moveCounts;

        robot.leftMotorBack.setTargetPosition(newLeftTargetBack);
        robot.rightMotorBack.setTargetPosition(newRightTargetBack);
        robot.leftMotorFront.setTargetPosition(newLeftTarget);
        robot.rightMotorFront.setTargetPosition(newRightTarget);

        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.leftMotorBack.setPower(speed);
        robot.rightMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(speed);
        robot.rightMotorFront.setPower(speed);
        while (opModeIsActive() &&
                (robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy()) && white < 1) {
            white = colorSensorGroundBack.alpha();
        }
// Stop all motion;
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorFront.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPowerLine(double speed) {
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(speed);
        robot.rightMotorFront.setPower(speed);
        robot.rightMotorBack.setPower(speed);
        double line = colorSensorGroundFront.alpha();
        while (line < .8 && opModeIsActive()) {
            line = colorSensorGroundFront.alpha();
        }
        setPower(0, 0);

        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}