package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Drop")
public class drop extends LinearOpMode {
    public DcMotor lift;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(1);
        waitForStart();
        while(opModeIsActive()){
            lift.setPower(-1);
            sleep(500);

        }
    }
}
