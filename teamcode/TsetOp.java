package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="TsetOp", group="SHS")

public class TsetOp extends OpMode{

    /* Declare OpMode members. */
    MarvHardware robot       = new MarvHardware();
    ColorSensor colorSensorBeacon;
    ColorSensor colorSensorGroundFront;
    ColorSensor colorSensorGroundBack;
    ModernRoboticsI2cGyro gyro  = null;
    ModernRoboticsI2cGyro gyro2  = null;


    @Override
    public void init() {
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro2 = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro2");
        colorSensorGroundBack = hardwareMap.colorSensor.get("color_sensor_ground_back");
        colorSensorGroundFront = hardwareMap.colorSensor.get("color_sensor_ground_front");
        colorSensorBeacon = hardwareMap.colorSensor.get("color_sensor_beacon");

        colorSensorBeacon.setI2cAddress(I2cAddr.create7bit(0x3E));
        colorSensorGroundBack.setI2cAddress(I2cAddr.create7bit(0x2E));
        colorSensorGroundFront.setI2cAddress(I2cAddr.create7bit(0x1E));
        gyro.setI2cAddress(I2cAddr.create7bit(0x18));
        gyro2.setI2cAddress(I2cAddr.create7bit(0x10));

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
    double beacon = 1;
    double lift = 0;
    double frontColor = 0;
    double backColor = 0;
    double beaconRed = 0;
    double beaconBlue = 0;
    double g = 0;
    double g2 = 0;
    double average = 0;


    @Override
    public void loop() {
        if (counter == 0) {
            eTime.reset();
            counter ++;
        }
       g =  gyro.getIntegratedZValue();
       g2 =  gyro2.getIntegratedZValue();
        double left;
        double right;

        double col;
        double con;

        boolean cs = false;
        boolean hs = false;


        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;



        if (gamepad2.dpad_up)
            col = 1;
        else if (gamepad2.dpad_down)
            col = -1;
        else
            col = 0;

        if (gamepad2.right_bumper) {
            lift = 1;
        } else if (gamepad2.left_bumper) {
            lift = -1;
        } else {
            lift = 0;
        }




        if (!(eTime.time() < 1.25) && !time) {
            if (gamepad2.y || gamepad1.y) {
                if (launcherOn) {
                    launcher = 0;
                    launcherOn = false;
                    eTime.reset();
                    time = false;
                } else {
                    launcher = -1;
                    launcherOn = true;
                    eTime.reset();
                    time = false;
                }

            }

        }

        if (gamepad1.a) {
            beacon = 1;
        } else if (gamepad1.b) {
            beacon = 0;
        }




        if (gamepad2.a && launcher == 0) {
            launcher = 1;
        } else if (!gamepad2.a && launcher == 1) {
            launcher = 0;
        }
        frontColor = colorSensorGroundFront.alpha();
        backColor = colorSensorGroundBack.alpha();
        beaconBlue = colorSensorBeacon.blue();
        beaconRed = colorSensorBeacon.red();


        average = (g + g2)/2;



        robot.leftMotorFront.setPower(left);
        robot.rightMotorFront.setPower(right);
        robot.leftMotorBack.setPower(left);
        robot.rightMotorBack.setPower(right);
        robot.leftLauncher.setPower(launcher);
        robot.rightLauncher.setPower(launcher);
        robot.lift.setPower(lift);
        robot.collector.setPower(col);
        robot.beacon.setPosition(beacon);

        telemetry.addData("Color Ground Front: ", frontColor);
        telemetry.addData("Color Ground Back: ", backColor);
        telemetry.addData("Beacon Red: ", beaconRed);
        telemetry.addData("Beacon Blue: ", beaconBlue);
        telemetry.addData("Gyro: ", g);
        telemetry.addData("Gyro 2:", g2);
        telemetry.addData("Average: ", average);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
