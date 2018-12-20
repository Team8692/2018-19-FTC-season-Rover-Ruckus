package org.firstinspires.ftc.teamcode.CompOne;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "run TELEOP")
@Disabled
public class tele extends OpMode {
    DcMotor left, right, intake1, intake2;
    CRServo lift1, lift2, lift3, lift4;
    Servo broom1, broom2;
    double servoPos = 1.0;
    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
		
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
		
        lift1 = hardwareMap.get(CRServo.class, "lift1");
        lift2 = hardwareMap.get(CRServo.class, "lift2");
        lift3 = hardwareMap.get(CRServo.class, "lift3");
        lift4 = hardwareMap.get(CRServo.class, "lift4");
        broom1 = hardwareMap.get(Servo.class, "broom1");
        broom2 = hardwareMap.get(Servo.class, "broom2");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        right.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(CRServo.Direction.REVERSE);
        lift4.setDirection(CRServo.Direction.REVERSE);
        broom2.setDirection(Servo.Direction.REVERSE);


    }

    @Override
    public void loop() {
		Drive(); 
		collect(); 
		elevator(); 
    }
	public void Drive(){
		double lefty = gamepad1.left_stick_y;
        double righty = gamepad1.right_stick_y;
        righty = Range.clip(righty, -1, 1);
        left.setPower(lefty);
        right.setPower(righty);

	}
	public void collect(){
		if (gamepad2.a) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        } else if (gamepad2.b) {
            intake1.setPower(1);
            intake2.setPower(1);
        } else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
	}
    public void elevator() {
        if (gamepad2.dpad_down) {
            lift1.setPower(1);
            lift2.setPower(1);
        } else if (gamepad2.dpad_up) {
            lift1.setPower(-1);
            lift2.setPower(-1);
        } else {
            lift1.setPower(0.0);
            lift2.setPower(0.0);
        }

        if (gamepad2.x) {
            lift3.setPower(1);
            lift4.setPower(1);
        } else if (gamepad2.y) {
            lift3.setPower(-1);
            lift4.setPower(-1);
        } else {
            lift3.setPower(0);
            lift4.setPower(0);
        }

        if (gamepad2.left_bumper) {
            servoPos = 1.0;
        } else if (gamepad2.right_bumper) {
            servoPos = 0.0;
        }
        broom1.setPosition(servoPos);
        broom2.setPosition(servoPos);
    }
}

