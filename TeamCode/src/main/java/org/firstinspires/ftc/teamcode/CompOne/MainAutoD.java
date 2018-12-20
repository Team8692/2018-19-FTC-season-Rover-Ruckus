package org.firstinspires.ftc.teamcode.CompOne;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

@Autonomous(name = "depot auto")
//@Disabled
public class MainAutoD extends BaseAuto {
    DcMotor left, right;
    public DcMotor leftb;
    public DcMotor rightb;
    public DcMotor intake;
    //public DcMotor intake2;
    public DcMotor lift;
    public Servo dump;

    public void runOpMode() throws InterruptedException {
        super.init();

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
       super.loop();

       GoToMinerals();

        dump.setPosition(1);
        sleep(500);
        dump.setPosition(0);

       super.stop();



    }

    @Override
    public void init() {
        super.init();
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        super.loop();
        try {
            GoToMinerals();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void GoToMinerals() throws InterruptedException {
        if(align==false) {
            left.setPower(-0.3);
            right.setPower(0.3);
        }
        if(align==true){
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
            sleep(200);
            left.setPower(0.5);
            right.setPower(0.5);
            leftb.setPower(0.5);
            rightb.setPower(0.5);
            sleep(4000);
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);

        }



    }

}
