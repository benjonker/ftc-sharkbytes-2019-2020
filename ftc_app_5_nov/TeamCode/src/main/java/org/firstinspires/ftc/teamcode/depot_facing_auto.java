package org.firstinspires.ftc.teamcode;

import android.graphics.NinePatch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="Depot Unhook Odometry", group="Odometry")
public class depot_facing_auto extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor mDrv_l0 = null;
    private DcMotor mDrv_r0 = null;
    private DcMotor mDrv_l1 = null;
    private DcMotor mDrv_r1 = null;
    private DcMotor mPin = null;
    private DcMotor mArm = null;
    private DcMotor mWch = null;
    private Servo sBox = null;

    private ElapsedTime runtime = new ElapsedTime();
    // TODO: explanation for all these box variables
    public final static double BOX_HOME = 0.0;
    public final static double BOX_MIN_HOME = 0.0;
    public final static double BOX_MAX_HOME = 1.0;
    public double boxPosition = BOX_HOME;
    //ColorSensor colorSensor; // not used

    public void stopMoving()
    {
        mDrv_l0.setPower(0);
        mDrv_l1.setPower(0);
        mDrv_r0.setPower(0);
        mDrv_r1.setPower(0);
        mArm.setPower(0);
        mWch.setPower(0);
        double sBoxStop = sBox.getPosition();
        sBox.setPosition(sBoxStop);
    }

    /* attempt to make a method that takes an array of motors and runs each motor for some amount of time... but java syntax sux2sux so re.
    public void runMotor(ArrayList motors, double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            for (DcMotor motor : motors) {
                motor.setPower(power);
            }
        }
        stopMoving();
    }*/

    public void turnLeft(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mDrv_l0.setPower(-power);
            mDrv_r0.setPower(power);
            mDrv_l1.setPower(-power);
            mDrv_r1.setPower(power);
        }
        stopMoving();
    }

    public void turnRight(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mDrv_l0.setPower(power);
            mDrv_r0.setPower(-power);
            mDrv_l1.setPower(power);
            mDrv_r1.setPower(-power);
        }
        stopMoving();
    }

    public void onlyLeft(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        mDrv_l0.setPower(power);
        mDrv_l1.setPower(power);
        while (opModeIsActive() && holdTimer.time() < holdTime) {}
        stopMoving();
    }

    public void onlyRight(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        mDrv_r0.setPower(power);
        mDrv_r1.setPower(power);
        while (opModeIsActive() && holdTimer.time() < holdTime) {}
        stopMoving();
    }

    public void forward (double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mDrv_l0.setPower(power);
            mDrv_r0.setPower(power);
            mDrv_l1.setPower(power);
            mDrv_r1.setPower(power);
        }
        stopMoving();
    }

    public void backward(double holdTime, double power)
    { // not used
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mDrv_l0.setPower(-power);
            mDrv_r0.setPower(-power);
            mDrv_l1.setPower(-power);
            mDrv_r1.setPower(-power);
        }
        stopMoving();
    }

    public void turnArm(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mArm.setPower(power);
        }
        mArm.setPower(0);
    }

    public void turnBoxServo(double holdTime, double turnAmount)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            boxPosition += turnAmount; // Adds the amount you want to turn the box to the current position.
            sBox.setPosition(boxPosition); // Sets the new position of the box
        }
    }
    public void setBoxPosition(double position)
    {
        while (opModeIsActive())
        {
            sBox.setPosition(position);
        }
    }

    public void raiseRackPinionMotor(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mPin.setPower(power);
        }
        mPin.setPower(0);
    }

    public void lowerRackPinionMotor(double holdTime, double power)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mPin.setPower(-power);
        }
        mPin.setPower(0);
    }

    public void extendArm(double holdTime, double power)
    { // not used
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mWch.setPower(power);
        }
    }

    public void extendRaise(double holdTime, double power)
    { // not used
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
        {
            mArm.setPower(power);
            mWch.setPower(power);
        }
    }

    /* code for sampling with color sensor, not used for this competition.
        public void jewel(double holdTime)
        {
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            //TODO: Figure out what colors we need to look for, and then assign what to do when we see them
            while (opModeIsActive() && holdTimer.time() < holdTime)
            {
                if (colorSensor.blue() > colorSensor.red()) //TODO: We need to get the rough RBG of each mineral
                {
                    turnRight(1, 1);
                    forward(0.5, 1);
                    telemetry.addData("BLUE", "%s visible");
                    //robot.armServo.setPosition(0.0);
                } else {
                    forward(0.5, 1);
                    turnLeft(1, 1);
                    //Assuming that when you turn right for 1 sec,
                    //and you turn back left for 1 you will be in the original position
                    telemetry.addData("RED", "%s visible");
                    //sArm.setPosition(ARM_HOME);
                }

            }
        }
    */

    public void runOpMode()
    {
        /*init color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        */
        // drive motors
        mDrv_l0 = hardwareMap.get(DcMotor.class, "mDrv_l0");
        mDrv_r0 = hardwareMap.get(DcMotor.class, "mDrv_r0");
        mDrv_l1 = hardwareMap.get(DcMotor.class, "mDrv_l1");
        mDrv_r1 = hardwareMap.get(DcMotor.class, "mDrv_r1");
        // other motors
        mPin = hardwareMap.get(DcMotor.class, "mPin");
        mArm = hardwareMap.get(DcMotor.class, "mArm");
        mWch = hardwareMap.get(DcMotor.class, "mWch");
        // servos
        sBox = hardwareMap.get(Servo.class, "sBox");

        // set motor directions so that positive "power" on motors moves the robot forwards
        mDrv_l0.setDirection(DcMotor.Direction.REVERSE);
        mDrv_l1.setDirection(DcMotor.Direction.REVERSE);

        // these variables are calibrated to the robot drive train for easier and more readable odometry
        double NINTEY_DEG = 0.585;
        double ONE_FOOT = 0.4;

        waitForStart();

        raiseRackPinionMotor(2, 1); // extend rack and pinion to lower robot
        turnRight(2*NINTEY_DEG, 1); // turns the robot 180 degrees to unhook
        forward(3*ONE_FOOT, 1); // drive forward into the depo
        extendRaise(1, 1); // turns the arm so that the token can be put down
        // the method turnArm might have to be used instead
        setBoxPosition(-1);
        turnRight(NINTEY_DEG+0.50, 1); // turn around to go towards crater
        forward(8.5*ONE_FOOT, 1); // go to the crater and park
        lowerRackPinionMotor(2, 1); // lowers the rack and pinion back into the robot
        extendRaise(1, 1); // puts the arm back
        // turnArm might have to be used instead
        setBoxPosition(1); // puts the box back

    }
}
