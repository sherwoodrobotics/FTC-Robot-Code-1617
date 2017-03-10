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
 * Joy 1:
 Tank Drive (4 wheel drive with Andy Mark motors)
 Two beacon servos (one with color sensor on it)

 Joy 2:
 2 motors, use tank drive for them
 2 motors independently controlled (4 buttons)
 2 servos (both controlled by same buttons, one forward and one back)
 2 servos (both controlled by same buttons, one forward and one back)

 Can you make all servos for both robots where they don’t go to a set position, but rather you hold and they move tornados that position?
 *
 */
public class NewCapHardware
{
    /* Public OpMode members. */

    public DcMotor  launcher1 = null;
    public DcMotor  launcher2 = null;
    public DcMotor  collector = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public NewCapHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        launcher1   = hwMap.dcMotor.get("lift_1");
        launcher2   = hwMap.dcMotor.get("lift_2");
        collector   = hwMap.dcMotor.get("collector");




        launcher1.setDirection(DcMotor.Direction.FORWARD);
        launcher2.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);


        launcher2.setPower(0);
        launcher1.setPower(0);
        collector.setPower(0);


        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




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

