package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kyledufrene on 2/19/17.
 */

@Autonomous(name = "MarvLaunchRamp", group = "SHS")

public class MarvLaunchRamp extends LinearOpMode {
    MarvHardware robot = new MarvHardware();
    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cGyro gyro2 = null;
    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro2 = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro2");
        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro.setI2cAddress(I2cAddr.create7bit(0x18));
        gyro2.setI2cAddress(I2cAddr.create7bit(0x10));
        gyro.calibrate();
        gyro2.calibrate();
        gyro.resetZAxisIntegrator();
        gyro2.resetZAxisIntegrator();
        while (!isStopRequested() && gyro.isCalibrating() && gyro2.isCalibrating()) {
            sleep(50);
            idle();
        }
        waitForStart();
        gyro.resetZAxisIntegrator();
        gyro2.resetZAxisIntegrator();

        robot.leftLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setMaxSpeed(1500);
        robot.rightLauncher.setPower(1);
        robot.leftLauncher.setPower(1);
        Wait(3);
        robot.collector.setPower(1);
        Wait(.15);
        robot.collector.setPower(0);
        Wait(.15);
        robot.collector.setPower(1);
        Wait(1);
        robot.collector.setPower(0);
        Wait(1);
        robot.leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLauncher.setPower(0);
        robot.leftLauncher.setPower(0);
        turn(1, -65);
        setPower(.5, .5);
        Wait(1);
        setPower(0, 0);
        turn(1, -180);
        setPower(.5, .5);
        Wait(3);
        setPower(0, 0);


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

    public void turn(double speed, double angle) {
        final double ERRORCOEFF = 1;
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
                setPower(-.65, .65);
            } else if (!direction && error < -30) {
                setPower(.65, -.65);
            } else if (direction && error > 22) {
                setPower(-.55, .55);
            } else if (!direction && error < -22) {
                setPower(.55, -.55);
            } else if (direction && error > 10) {
                setPower(-.4, .4);
            } else if (!direction && error < -10) {
                setPower(.4, -.4);
            } else if (direction && error > 1) {
                setPower(-.35, .35);
            } else if (!direction && error < -1) {
                setPower(.35, -.35);
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

    private double getGyroAverage() {
        double g1 = gyro.getIntegratedZValue();
        double g2 = gyro2.getIntegratedZValue();

        return ((g1 + g2) / 2);
    }
}