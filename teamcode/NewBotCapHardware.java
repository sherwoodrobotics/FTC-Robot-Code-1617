package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kyledufrene on 2/18/17.
 */

public class NewBotCapHardware {
    public Servo Switch = null;
    HardwareMap hwMap = null;

    private ElapsedTime period  = new ElapsedTime();
    public NewBotCapHardware() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        Switch = hwMap.servo.get("switch");


    }
}