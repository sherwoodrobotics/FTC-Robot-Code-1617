package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareLauncherBot
{
    /* Public OpMode members. */
    public DcMotor  leftMotorFront   = null;
    public DcMotor  rightMotorFront  = null;
    public DcMotor  leftMotorBack   = null;
    public DcMotor  rightMotorBack  = null;
    public DcMotor  leftLauncher    = null;
    public DcMotor  rightLauncher    = null;
    public DcMotor  collector    = null;
    public DcMotor  conveyor    = null;
    public Servo    hitSensor    = null;
    public Servo    colorSensor    = null;

    public static final double hitServo       =  0;
    public static final double colorServo       =  0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareLauncherBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFront   = hwMap.dcMotor.get("left_drive_front");
        rightMotorFront  = hwMap.dcMotor.get("right_drive_front");
        leftMotorBack   = hwMap.dcMotor.get("left_drive_back");
        rightMotorBack  = hwMap.dcMotor.get("right_drive_back");
        leftLauncher    = hwMap.dcMotor.get("left_launcher");
        rightLauncher    = hwMap.dcMotor.get("right_launcher");
        collector    = hwMap.dcMotor.get("collector");
        conveyor    = hwMap.dcMotor.get("conveyor");
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftLauncher.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightLauncher.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        collector.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        conveyor.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        collector.setPower(0);
        conveyor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        hitSensor = hwMap.servo.get("hit_sensor");
        colorSensor = hwMap.servo.get("color_sensor");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

