package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kyledufrene on 2/19/17.
 */

@Autonomous(name = "MarvLaunchCenter", group = "SHS")

public class MarvLaunchCenter extends LinearOpMode {
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
        Wait(.15);
        robot.collector.setPower(0);
        Wait(1);
        robot.collector.setPower(1);
        Wait(1);
        robot.collector.setPower(0);
        Wait(1);
        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setPower(0);
        robot.leftLauncher.setPower(0);
        setPower(.5,.5);
        Wait(2.5);
        setPower(0,0);



    }

    public void setPower(double left, double right) {
        robot.leftMotorBack.setPower(left);
        robot.leftMotorFront.setPower(left);
        robot.rightMotorFront.setPower(right);
        robot.rightMotorBack.setPower(right);
    }


    public void Wait(double time) {
        eTime.reset();
        while (eTime.time() < time && opModeIsActive()) {
        }
    }
}