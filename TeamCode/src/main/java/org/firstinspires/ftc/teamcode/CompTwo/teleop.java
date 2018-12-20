package org.firstinspires.ftc.teamcode.CompTwo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CompTwo.Robot;

@TeleOp(name = "TELEOP")
public class teleop extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            Drive();
            Collect();
            Latch();
        }
    }

    public void Drive() {
        double lefty = -gamepad1.left_stick_y;
        double righty = -gamepad1.right_stick_y;

        righty = Range.clip(Math.pow(righty,3), -1, 1);
        lefty = Range.clip(Math.pow(lefty,3), -1, 1);

        left.setPower(lefty);
        right.setPower(righty);
        leftb.setPower(lefty);
        rightb.setPower(righty);
    }

    public void Collect() {
        if (gamepad2.right_trigger > 0) {
            intake.setPower(1);
            //intake2.setPower(1);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-1);
            //intake2.setPower(-1);
        } else {
            intake.setPower(0);
            //intake2.setPower(0);
        }
    }

    public void Latch() {
        if (gamepad2.left_stick_y > 0) {
            //lift.setPower(1);
        } else if (gamepad2.left_stick_y < 0) {
            //lift.setPower(-1);
        } else {
            //lift.setPower(0);
        }

        if(gamepad1.a){
            dump.setPosition(1);

        }else if(gamepad1.b){
            dump.setPosition(0);
        }
    }
}