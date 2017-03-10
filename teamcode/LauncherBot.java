package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@TeleOp(name="SHS: LauncherBot", group="SHS")

public class LauncherBot extends OpMode{

    /* Declare OpMode members. */
    HardwareLauncherBot robot       = new HardwareLauncherBot();



    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello SHS Driver");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    boolean launcherOn = false;
    double launcher = 0;
    ElapsedTime eTime = new ElapsedTime();
    int counter = 0;
    boolean time = false;


    @Override
    public void loop() {
        if (counter == 0) {
            eTime.reset();
            counter ++;
        }
        double left;
        double right;

        double col;
        double con;

        boolean cs = false;
        boolean hs = false;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        //ACTUALLY, don't negate it. Andy Mark motors are opposite of tetrix.
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;


        if (gamepad1.b)
            robot.colorSensor.setPosition(.78431);
        if (gamepad1.a)
            robot.colorSensor.setPosition(.117647058);
        if (gamepad1.x)
            robot.hitSensor.setPosition(.21568);
        if (gamepad1.y)
            robot.hitSensor.setPosition(.803921);

        if (gamepad2.dpad_up)
            col = 1;
        else if (gamepad2.dpad_down)
            col = -1;
        else
            col = 0;

        if (gamepad2.right_trigger != 0)
            con = -1;
        else if (gamepad2.left_trigger != 0)
            con = 1;
        else
            con = 0;




        if (!(eTime.time() < 1.25) && !time) {
            if (gamepad2.y) {
                if (launcherOn) {
                    launcher = 0;
                    launcherOn = false;
                    eTime.reset();
                    time = false;
                } else {
                    launcher = 1;
                    launcherOn = true;
                    eTime.reset();
                    time = false;
                }

            }

        }






        if (gamepad2.a && launcher == 0) {
            launcher = -1;
        } else if (!gamepad2.a && launcher == -1) {
            launcher = 0;
        }





        robot.leftMotorFront.setPower(left);
        robot.rightMotorFront.setPower(right);
        robot.leftMotorBack.setPower(left);
        robot.rightMotorBack.setPower(right);
        robot.leftLauncher.setPower(launcher);
        robot.rightLauncher.setPower(launcher);
        robot.collector.setPower(col);
        robot.conveyor.setPower(con);


        // Send telemetry message to signify robot running;
        telemetry.addData("hit sensor servo ",  "%.2b", hs);
        telemetry.addData("color sensor servo ",  "%.2b", cs);
        telemetry.addData("left drive ",  "%.2f", left);
        telemetry.addData("right drive ", "%.2f", right);
        telemetry.addData("launcher ",  "%.2f", launcher);
        telemetry.addData("collector ",  "%.2f", col);
        telemetry.addData("conveyor ", "%.2f", con);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
