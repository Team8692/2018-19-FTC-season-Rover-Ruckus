package org.firstinspires.ftc.teamcode.SAFE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RUN THIS TELEOP")
public class mecaControl extends teleop {
    @Override void drive() {
        double sidwaysPow = -gamepad1.left_stick_x;
        double forwardPow = -gamepad1.left_stick_y;
        double turnPow = -gamepad1.right_stick_x;

        //converts xy coordinates to polar coordinates
        double r = Math.hypot(sidwaysPow, forwardPow);
        //calculates the current heading of the robot
        double robotAngle = Math.atan2(forwardPow, sidwaysPow) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle) + turnPow;
        double v2 = r * Math.sin(robotAngle) - turnPow;
        double v3 = r * Math.sin(robotAngle) + turnPow;
        double v4 = r * Math.cos(robotAngle) - turnPow;
        v1 = Range.clip(v1, -.33, .33);
        v2 = Range.clip(v2, -.33, .33);
        v3 = Range.clip(v3, -1, 1);
        v4 = Range.clip(v4, -1, 1);

        left.setPower(v1);
        right.setPower(v2);
        leftb.setPower(v3);
        rightb.setPower(v4);
    }
}