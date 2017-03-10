package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="MarvDrive", group="SHS")

public class MarvDrive extends OpMode{

    /* Declare OpMode members. */
    MarvHardware robot       = new MarvHardware();



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
    boolean armsHolded = true;
    double launcher = 0;
    ElapsedTime eTime = new ElapsedTime();
    int counter = 0;
    boolean time = false;
    double beacon = 1;
    double lift = 0;

    double capFolded = .5599;
    double capOpen = .4;
    double capPos = capFolded;


    double leftLock = .28;
    double leftOpen = .4;
    double leftHoldPos = leftLock;

    double rightLock = .25;
    double rightOpen = .1;
    double rightHoldPos = rightLock;

    @Override
    public void loop() {
        if (counter == 0) {
            eTime.reset();
            counter ++;
        }
        double left;
        double right;
        double col;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        //Collector: gamepad2 dpad up,down
        if (gamepad2.dpad_up)
            col = 1;
        else if (gamepad2.dpad_down)
            col = -1;
        else
            col = 0;

        //Cap Ball system: gamepad2 bumpers
        if (gamepad2.left_bumper && capPos <= capFolded) {
            capPos += .001;
        }
        if (gamepad2.right_bumper && capPos >= capOpen) {
            capPos -= .001;
        }

        //Lift: gamepad2, triggers
        if (gamepad2.right_trigger >= 0.5) {
            lift = 1;
        } else if (gamepad2.left_trigger >= 0.5) {
            lift = -1;
        } else {
            lift = 0;
        }

        //Launcher: gamepad2 a, y;  gamepad1 y
        if (!(eTime.time() < 1.25) && !time) {
            if (gamepad2.y || gamepad1.y) {
                if (launcherOn) {
                    unlaunch();
                    launcherOn = false;
                    eTime.reset();
                    time = false;
                } else {
                    launch();
                    launcherOn = true;
                    eTime.reset();
                    time = false;
                }
            }
        }
        if (gamepad2.a && launcher == 0) {
            launcher = 1;
        } else if (!gamepad2.a && launcher == 1) {
            launcher = 0;
        }

        //Beacon servo: gamepad1 a, b
        if (gamepad1.a) {
            beacon = 1;
        } else if (gamepad1.b) {
            beacon = 0;
        }

        //Hold controls
        if (!(eTime.time() < 1) && !time) {
            if (gamepad2.x || gamepad1.x) {
                if (armsHolded) {
                    leftHoldPos = leftOpen;
                    rightHoldPos = rightOpen;
                    armsHolded = false;
                    eTime.reset();
                    time = false;
                } else {
                    leftHoldPos = leftLock;
                    rightHoldPos = rightLock;
                    armsHolded = true;
                    eTime.reset();
                    time = false;
                }
            }
        }



        robot.leftMotorFront.setPower(left);
        robot.rightMotorFront.setPower(right);
        robot.leftMotorBack.setPower(left);
        robot.rightMotorBack.setPower(right);
        //robot.leftLauncher.setPower(launcher);
        //robot.rightLauncher.setPower(launcher);
        robot.lift.setPower(lift);
        robot.cap.setPosition(capPos);
        robot.collector.setPower(col);
        robot.beacon.setPosition(beacon);
        robot.leftHold.setPosition(leftHoldPos);
        robot.rightHold.setPosition(rightHoldPos);
        telemetry.addData("Cap ", capPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void launch() {
        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setPower(1);
        robot.leftLauncher.setPower(1);
    }

    public void unlaunch() {
        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setPower(0);
        robot.leftLauncher.setPower(0);
    }
}
