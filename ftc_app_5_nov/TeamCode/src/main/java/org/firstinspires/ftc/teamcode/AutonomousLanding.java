package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="AutonomousLanding")
//@Disabled
public class AutonomousLanding extends LinearOpMode {
    private Servo sPin = null; // servo on port zero

    @Override
    public void runOpMode() {

        sPin = hardwareMap.get(Servo.class, "sPin"); // get the port thingy

        waitForStart();

        while (opModeIsActive()) {
            float direction = (-gamepad1.right_stick_y);

            if (direction > 0.1) {
                sPin.setPosition(direction);
            }
            if (direction < -0.1) {
                sPin.setPosition(direction);
            }
        }
    }
}

