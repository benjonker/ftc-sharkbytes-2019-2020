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
    private Servo sBox = null;

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
        double sArmStop = sArm.getPosition();
        sArm.setPosition(sArmStop);
        double sBoxStop = sBox.getPosition();
        sBox.setPosition(sBoxStop);

    }

    public void turnLeft(double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && 0 < holdTime) {
            mDrv_l0.setPower(power);
            mDrv_r0.setPower(-power);
            mDrv_l1.setPower(power);
            mDrv_r1.setPower(-power);
        }
        stopMoving();
    }

    public void turnRight(double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(-power);
            mDrv_r0.setPower(power);
            mDrv_l1.setPower(-power);
            mDrv_r1.setPower(power);
        }
        stopMoving();
    }

    public void forward (double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(power);
            mDrv_r0.setPower(power);
            mDrv_l1.setPower(power);
            mDrv_r1.setPower(power);
        }
        stopMoving();
    }

    public void backward(double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mDrv_l0.setPower(-power);
            mDrv_r0.setPower(-power);
            mDrv_l1.setPower(-power);
            mDrv_r1.setPower(-power);
        }
        stopMoving();
    }

    public void turnArmServo(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            sArm = hardwareMap.get(Servo.class, "arm");
            armPosition += ARM_SPEED;
            sArm.setPosition(armPosition);
        }

    }

    public void raiseRackPinionMotor(double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            mPin.setPower(power);
        }
    }

    public void lowerRackPinionMotor(double holdTime, double power) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
           mPin.setPower(-power);
        }
    }

    public void jewel(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //TODO: Figure out what colors we need to look for, and then assign what to do when we see them
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (colorSensor.blue() > colorSensor.red()) { //TODO: We need to get the rough RBG of each jewel and gold coin
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
                sArm.setPosition(ARM_HOME);
            }

        }
    }

    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        mDrv_l0 = hardwareMap.get(DcMotor.class, "mDrv_l0");
        mDrv_r0 = hardwareMap.get(DcMotor.class, "mDrv_r0");
        mDrv_l1 = hardwareMap.get(DcMotor.class, "mDrv_l1");
        mDrv_r1 = hardwareMap.get(DcMotor.class, "mDrv_l2");
        mPin = hardwareMap.get(DcMotor.class, "mPin");
        sArm = hardwareMap.get(Servo.class, "sArm");
        sBox = hardwareMap.get(Servo.class, "sBox");

        mDrv_l1.setDirection(DcMotor.Direction.REVERSE);
        mDrv_r1.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        //Unhooking the robot.
        raiseRackPinionMotor(5, 1);
        backward(2, 1);
        lowerRackPinionMotor(5, 1);

        jewel(5);
        //Naming the variable holdTime is misleading. You should change this Livi.
    }
}