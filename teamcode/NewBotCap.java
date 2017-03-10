package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
/**
 * Created by kyledufrene on 2/18/17.
 */
@TeleOp(name="SHS: NewBotCap", group="SHS")

/* Created by wyatt.ross on 9/22/2015.
*/
public class NewBotCap extends OpMode {
    NewBotCapHardware robot       = new NewBotCapHardware();
    ElapsedTime eTime = new ElapsedTime();

    public NewBotCap() {
    }
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.Switch.setPosition(FOLDED_POS);

    }
    public static final double FOLDED_POS = .55;
    public static final double OPEN_POS = .405;

    double position = FOLDED_POS;

    @Override
    public void loop() {
        //p2 = robot.Switch.getPosition();

        /**
        if (gamepad2.left_bumper) {
            robot.Switch.setPosition(.1);
        }
        else if(gamepad2.right_bumper) {robot.Switch.setPosition(0.2);}
        else if (gamepad2.a) {robot.Switch.setPosition(.5);}
        else if (gamepad2.b) {robot.Switch.setPosition(.8);}
        else {
            robot.Switch.setPosition(.1);

        }

         */

        telemetry.addData("Servo Position: ", position);

        if (gamepad2.left_bumper && position <= FOLDED_POS) {
            position += .001;
        }
        if (gamepad2.right_bumper && position >= OPEN_POS) {
            position -= .001;
        }

        robot.Switch.setPosition(position);

    }

}

