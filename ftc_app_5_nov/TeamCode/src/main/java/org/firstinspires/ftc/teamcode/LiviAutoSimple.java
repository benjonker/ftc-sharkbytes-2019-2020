package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoSample_Linear", group="Chris")
public class LiviAutoSimple extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        aDrive =  hardwareMap.get(DcMotor.class, "mDrv_l0");
        bDrive = hardwareMap.get(DcMotor.class, "mDrv_r0");

        //TEST
        aDrive.setDirection(DcMotor.Direction.FORWARD);
        cDrive.setDirection(DcMotor.Direction.FORWARD);


    }
    public void turnLeft(){
        telemetry.addData("status" , "turnLeft"); // Is the bot aligned with the gold mineral
        telemetry.update();


        aDrive.setPower(0.1);
        bDrive.setPower(-0.1);

    }

    public void turnRight(){
        telemetry.addData("status" , "turnRight"); // Is the bot aligned with the gold mineral
        telemetry.update();


        aDrive.setPower(-0.1);
        bDrive.setPower(0.1);

    }

    public void forward(){
        telemetry.addData("status" , "forward"); // Is the bot aligned with the gold mineral
        telemetry.update();

        aDrive.setPower(0.8);
        bDrive.setPower(0.8);

    }


}