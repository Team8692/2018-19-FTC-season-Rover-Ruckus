package org.firstinspires.ftc.teamcode.SAFE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tank TELEOP")
@Disabled
public class teleop extends Robot {
    boolean mecanum = false;

    @Override public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        while(opModeIsActive()) {
            drive();
            latch();
        }
    }

    void drive() {
        double lefty = -gamepad1.left_stick_y;
        double righty = -gamepad1.right_stick_y;

        righty = Range.clip(righty, -1, 1);
        lefty = Range.clip(lefty, -1, 1);

        left.setPower(lefty);
        right.setPower(righty);
        leftb.setPower(lefty);
        rightb.setPower(righty);
    }

    void latch() {
        if (gamepad2.right_trigger > 0) {
            lift.setPower(1);

        } else if (gamepad2.left_trigger > 0) {
            lift.setPower(-1);

        } else {
            lift.setPower(0);

        }

        if (gamepad2.right_bumper) {

            lift2.setPower(1);
        } else if (gamepad2.left_bumper) {

            lift2.setPower(-1);
        } else {

            lift2.setPower(0);
        }

        if(gamepad2.a){
            hook.setPosition(1);
        }else if(gamepad2.b){
            hook.setPosition(0);
        }

//        double hooky = gamepad2.left_stick_y;
//        hooky = Range.clip(hooky, 0, 1);
//
//        hook.setPosition(hooky);

    }


}
