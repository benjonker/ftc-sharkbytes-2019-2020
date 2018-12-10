/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController; // doing a weirdo so now using DcMotor.RunMode... is this deprecated? Dec 8 2018
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="full robot DEV", group="Linear Opmode")
//@Disabled
public class teleop_dev extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare the drive motors with our naming conventions
    private DcMotor mDrv_r0 = null; // left front
    private DcMotor mDrv_r1 = null; // left back
    private DcMotor mDrv_l0 = null; // right front
    private DcMotor mDrv_l1 = null; // right back

    private DcMotor mPin = null; // rack and pinion motor

    private DcMotor mArm = null; // arm base servo
    private DcMotor mWch = null; // arm extending motor
    private Servo sBox = null; // box that holds shizz

    boolean aPressed = false; // checks if the a button is pressed, which stops the box servo

    /* Albert's functions (deprecated)
        public void powerRight(double power) { // set power to the right motors
            mDrv_r0.setPower(power);
            mDrv_r1.setPower(power);
        }

        public void powerLeft(double power) { // set power to the left motors
            mDrv_l0.setPower(power);
            mDrv_l1.setPower(power);
        }
    */
    public void power(double leftPower, double rightPower, double pinionPower, double boxPower, double armPower, double wchPower)
    {
        //This int check is declared and is used the check whether the box is manually turned or not. If it is manually turned, it = 1.
        //Otherwise, it = 0.
        boolean check = false;
        aPressed = false;
        mDrv_r0.setPower(rightPower);
        mDrv_r1.setPower(rightPower);
        mDrv_l0.setPower(leftPower);
        mDrv_l1.setPower(leftPower);
        mPin.setPower(pinionPower);
        mWch.setPower(wchPower);
        //I dunno if this works. sBox.getPowerFloat(); is something you can do, but i dunno how to work that.
        if (gamepad2.a)
        {
            check = true;
            sBox.setPosition(0);
        }
        else if (boxPower > 0.25)
        {
            sBox.setPosition(boxPower);
            check = true;
        }
        else if (boxPower < -0.25)
        {
            sBox.setPosition(boxPower);
            check = true;
        }
        else
        {
            sBox.setPosition(0);
        }
        if (armPower > 0.1)
        {
            //mArm.setPower(armPower);
            if (!check)
            {
                sBox.setPosition(-armPower);
            }
        }
        else if (armPower < -0.1)
        {
            //mArm.setPower(armPower);
            // TODO: these arm setpowers are commented out because they are being replaced by a thing in the main loop that just makes the thing go up if the joystick is up else down
            if (!check)
            {
                sBox.setPosition(-armPower);
            }
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // drive motors
        mDrv_r0  = hardwareMap.get(DcMotor.class, "mDrv_r0");
        mDrv_r1  = hardwareMap.get(DcMotor.class, "mDrv_r1");
        mDrv_l0  = hardwareMap.get(DcMotor.class, "mDrv_l0");
        mDrv_l1  = hardwareMap.get(DcMotor.class, "mDrv_l1");
        // rack and pinion, and intake
        mPin = hardwareMap.get(DcMotor.class, "mPin"); // rack and pinion motor
        // arm motors and servos
        mArm = hardwareMap.get(DcMotor.class, "mArm"); // arm base motor
        mWch = hardwareMap.get(DcMotor.class, "mWch"); // arm extending motor
        sBox = hardwareMap.get(Servo.class, "sBox"); // box that holds shizz

        mDrv_l0.setDirection(DcMotor.Direction.REVERSE);
        mDrv_l1.setDirection(DcMotor.Direction.REVERSE);
        mDrv_r0.setDirection(DcMotor.Direction.FORWARD);
        mDrv_r1.setDirection(DcMotor.Direction.FORWARD);

        mWch.setDirection(DcMotor.Direction.FORWARD);

        // encoders:
        /**
         * encoders:
         * four ways to run them:
         * motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); // default
         * motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS); // you can set speed of the motor
         * motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION); // turns motor into a to a servo
         * motor.setMode(DcMotorController.RunMode.RESET_ENCODERS); // "reset encoder value to zero" ??????????
         */
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder to "position 0"

        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // set the mode to run to position

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // these lines should be reversed because we always have right above left but sike its not so sux2sux
            double leftPower; // this is the describer variable for storing the power of all the left drive motors
            double rightPower; // same as above but for the right side

            double pinPower; // power for the pinion motor
            double boxPower; // for the box servo
            double armPower; // for the servo at the base of the arm
            double wchPower; //for the motor that extends the arm


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            rightPower = -gamepad1.right_stick_y; // set the powers with the game sticks every frame
            leftPower  = -gamepad1.left_stick_y;  // same as above but not

            pinPower = gamepad2.left_stick_y;
            boxPower = -gamepad2.left_stick_x;

            wchPower = -gamepad2.right_stick_x;
            armPower = -gamepad2.right_stick_y;


            // nicassa's function -- this sets power to the motors, but not to the arm motor, this is taken care of below
            power(leftPower, rightPower, pinPower, boxPower, armPower, wchPower);

            // arm motor
            mArm.setTargetPosition((int)(armPower)); // run the motor to the position of

            // Show the elapsed game time and wheel power.
            // telemetry is that little black console below all the controls that appears while the robot is running
           // telemetry.addData("Status", "Run Time: " + runtime.toString());
            /*
            telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("motors", "Intake (%.2f)\nPinion (%.2f)", 0, pinPower);
            telemetry.addData("arm", "Arm (%.2f), Winch (%.2f)", armPower, wchPower);
            */
            //telemetry.addData("arm position by encoder", armPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Box", "(%.2f)", boxPower);
            telemetry.addData("Rack and Pinion", "(%.2f)", pinPower);
            telemetry.addData("Arm", "Arm (%.2f), Winch (%.2f), Box (%.2f)", armPower, wchPower, boxPower);
            if (aPressed)
            {
                telemetry.addLine("A is pressed");
            }
            else
            {
                telemetry.addLine("A is not pressed");
            }
            telemetry.update();
        }
    }
}
