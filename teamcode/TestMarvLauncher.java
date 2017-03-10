package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kyledufrene on 2/19/17.
 */

@Autonomous(name = "TestMarvLauncher", group = "SHS")

public class TestMarvLauncher extends LinearOpMode {
    MarvHardware robot = new MarvHardware();

    ElapsedTime eTime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


        robot.leftLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setPower(1);
        robot.leftLauncher.setPower(1);
        Wait(3);
        robot.collector.setPower(1);
        Wait(15);

        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setPower(0);
        robot.leftLauncher.setPower(0);

    }

    public void Drive(double speedR, double speedL) {
        robot.leftMotorBack.setPower(speedL);
        robot.leftMotorFront.setPower(speedL);
        robot.rightMotorBack.setPower(speedR);
        robot.rightMotorFront.setPower(speedR);
    }

    public void Wait(double time) {
        eTime.reset();
        while (eTime.time() < time && opModeIsActive()) {
        }
    }
}