package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Crater Only")
@Disabled
public class crater extends autonomus {
    int statec = 1;

    @Override
    public void handleAutoLoop() {
        while (opModeIsActive()) {
            switch (statec) {


                case 1:
                    minerals();
                    statec++;
                    break;
                case 2:
                    crater();
                    statec++;
                    break;
            }
        }
    }

    @Override
    public void crater() {
        robot.drive(0.75,5,5);
    }

    public void minerals() {
        robot.activateTenserFlow();
        //wait for start up
        sleep(500);
        robot.runTenserFlow();

        //distance between minerals: 14.5in, distance from robot to middle mineral: 23.5in
        //angle from robot to left or right minerals is arctan(14.5/23.5)=32deg
        //distance from robot to left or right minerals is sqrt(14.5^2+23.5^2)=27.6
        if (robot.check==true) {

            robot.left.setPower(1);
            robot.leftb.setPower(1);
            robot.right.setPower(1);
            robot.rightb.setPower(1);
            sleep(3000);
            robot.left.setPower(0);
            robot.leftb.setPower(0);
            robot.right.setPower(0);
            robot.rightb.setPower(0);
        } else {
            while(robot.check==false) {
                robot.left.setPower(.1);
                robot.leftb.setPower(.1);
                robot.right.setPower(-.1);
                robot.rightb.setPower(-0.1);

            }
        }
    }
}
