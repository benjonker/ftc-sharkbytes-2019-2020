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

    private DcMotor mPin = null; // motor used for pinion

    @Override
    public void runOpMode()
    {
        mPin = hardwareMap.get(DcMotor.class, "mPin"); // gets the port

        waitForStart();

        while (opModeIsActive())
        {
        }
    }
    //This whole script is jut for the method below.
    //This method extends the robot to the floor, moves it slightly to the right or left,
    // and the de-extends the rack and pinion back into the robot.
    public void robotLanding()
    {
        mPin = hardwareMap.get(DcMotor.class, "mPin"); // gets the port
        //The amount of time will is tbd, for now I have set it to two seconds
        int amountOfTimeToExtendPinion = 4;
        int frameCount = 0;
        int seconds = 0;
        while (seconds > amountOfTimeToExtendPinion)
        {
            mPin.setPower(1);
            frameCount++;
            seconds = frameCount/60;
        }
        //Run a method that moves the robot to the left or right
        frameCount = 0;
        seconds = 0;
        while (seconds > amountOfTimeToExtendPinion)
        {
            mPin.setPower(-1);
            frameCount++;
            seconds = frameCount/60;
        }
        //The while statement above isn't necessary, it just puts the pinion back inside the robot.
    }
}

