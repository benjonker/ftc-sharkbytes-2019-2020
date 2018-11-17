package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

//TODO: We need to test this code, and get vuforia working.

@Autonomous(name="AutoSample_Linear", group="Chris")
public class LiviAutoSimple extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor mDrv_l0 = null;
    private DcMotor mDrv_r0 = null;
    private DcMotor mDrv_l1 = null;
    private DcMotor mDrv_r1 = null;
    private DcMotor mPin = null;
    private Servo sArm = null;
    //private Servo sBox = null;

    private ElapsedTime runtime = new ElapsedTime();

    public final static double ARM_HOME = 0.0;
    public final static double ARM_MIN_HOME = 0.0;
    public final static double ARM_MAX_HOME = 1.0;
    public final static double ARM_SPEED = 0.1;
    public double armPosition = ARM_HOME;
    ColorSensor colorSensor;


    public void stopMoving() {
        mDrv_l0.setPower(0);
        mDrv_l1.setPower(0);
        mDrv_r0.setPower(0);
        mDrv_r1.setPower(0);
    }

    public void turnLeft(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(0.1);
            mDrv_r0.setPower(-0.1);
            mDrv_l1.setPower(0.1);
            mDrv_r1.setPower(-0.1);
        }
        stopMoving();

    }

    public void turnRight(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(-0.1);
            mDrv_r0.setPower(0.1);
            mDrv_l1.setPower(-0.1);
            mDrv_r1.setPower(0.1);
        }
        stopMoving();
    }

    public void forward (double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(0.8);
            mDrv_r0.setPower(0.8);
            mDrv_l1.setPower(0.8);
            mDrv_r1.setPower(0.8);
        }
        stopMoving();
    }

    public void backward(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(-0.8);
            mDrv_r0.setPower(-0.8);
            mDrv_l1.setPower(-0.8);
            mDrv_r1.setPower(-0.8);
        }
        stopMoving();
    }

    public void turnServo(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            sArm = hardwareMap.get(Servo.class, "arm");
            armPosition += ARM_SPEED;
            sArm.setPosition(armPosition);
        }

    }

    public void raiseRackPinionMotor(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mPin.setPower(1);
        }
    }

    public void lowerRachPinionMotor(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
           mPin.setPower(-1);
        }
    }

    public void jewel(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //TODO: Figure out what colors we need to look for, and then assign what to do when we see them
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (colorSensor.blue() > colorSensor.red()) { //TODO: We need to get the rough RBG of each jewel and gold coin
                turnRight(1);
                forward(0.5);
                telemetry.addData("BLUE", "%s visible");
                //robot.armServo.setPosition(0.0);
            } else {
                forward(0.5);
                turnLeft(1);
                //Assuming that when you turn right for 1 sec,
                //and you turn back left for 1 you will be in the original position
                telemetry.addData("RED", "%s visible");
                sArm.setPosition(ARM_HOME);
            }

        }
    }

    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        mDrv_l0 = hardwareMap.get(DcMotor.class, "mDrv_l0");
        mDrv_r0 = hardwareMap.get(DcMotor.class, "mDrv_r0");
        mPin = hardwareMap.get(DcMotor.class, "mPin");
        sArm = hardwareMap.get(Servo.class, "sArm");

        mDrv_l1.setDirection(DcMotor.Direction.REVERSE);
        mDrv_r1.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        //Unhooking the robot.
        raiseRackPinionMotor(5);
        backward(2);
        lowerRachPinionMotor(5);

        jewel(5);


        }
    }


}