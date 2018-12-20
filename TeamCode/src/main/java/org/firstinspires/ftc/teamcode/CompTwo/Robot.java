package org.firstinspires.ftc.teamcode.CompTwo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends LinearOpMode {
    public DcMotor left;
    public DcMotor right;
    public DcMotor leftb;
    public DcMotor rightb;
    public DcMotor intake;
    //public DcMotor intake2;
    //public DcMotor lift;
    public Servo dump; //to radians 2. get arc length (does not need to multiply by 2, since robot is symmetrical) 3. drive calculated arc length


    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        leftb = hardwareMap.get(DcMotor.class, "leftb");
        rightb = hardwareMap.get(DcMotor.class, "rightb");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake2 = hardwareMap.get(DcMotor.class, "intake2");
        //lift = hardwareMap.get(DcMotor.class, "lift");
        dump = hardwareMap.get(Servo.class, "dump");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void ini(HardwareMap hardwareMap) {

    }
}