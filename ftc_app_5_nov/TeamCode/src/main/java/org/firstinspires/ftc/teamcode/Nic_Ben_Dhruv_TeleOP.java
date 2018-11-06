/* IMPORTANT: On the first controller left and right joysticks are used to move the robot.
You push the left joystick forward and right joystick back to go to the left,
right joystick forward and left joystick back to go right,
both forward to go forward,
and both backwards to go back.
On the second controller you use the left joystick for the pinion,
and the right joystick for the intake.
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Nic_Ben_Dhruv_TeleOP", group="TeleOpmode")
public class Nic_Ben_Dhruv_TeleOP extends LinearOpMode
{
    public DcMotor mDrv_l; //Left motor
    public DcMotor mDrv_r; //Right motor
    public DcMotor mDrv_l2; //left motor 2
    public DcMotor mDrv_r2; //right motor 2
    public DcMotor mLPinion; //Pinion motor
    public DcMotor mRIntake; //Intake motor

    @Override
    //@Disabled
    public void runOpMode() throws InterruptedException
    {
        mDrv_l = hardwareMap.dcMotor.get("mDrv_l");
        mDrv_r = hardwareMap.dcMotor.get("mDrv_r");
        mDrv_l2 = hardwareMap.dcMotor.get("mDrv_l2");
        mDrv_r2 = hardwareMap.dcMotor.get("mDrv_r2");
        mLPinion = hardwareMap.dcMotor.get("mLPinion");
        mRIntake = hardwareMap.dcMotor.get("mRIntake");

        mDrv_l.setDirection(DcMotor.Direction.REVERSE); //Most robots have one motor backwards
        mDrv_l2.setDirection(DcMotor.Direction.REVERSE); //Most robots have one motor backwards

        waitForStart();

        while(opModeIsActive())
        {
            mDrv_l.setPower(-gamepad1.left_stick_y);
            mDrv_r.setPower(-gamepad1.right_stick_y);
            mDrv_l2.setPower(-gamepad1.left_stick_y);
            mDrv_r2.setPower(-gamepad1.right_stick_y);
            mLPinion.setPower(-gamepad2.left_stick_y);
            mRIntake.setPower(-gamepad2.right_stick_y);

            idle();
        }
    }
}