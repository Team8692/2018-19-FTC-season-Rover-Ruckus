package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;

//import android.content.res.Configuration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name = "Full Auto")
@Disabled
public class autonomus extends LinearOpMode {

        int state = 0;

        RobotAuto robot = new RobotAuto();

        @Override
        public void runOpMode() throws InterruptedException {
            robot.injectHW(this);
            robot.runOpMode();

            robot.initCam();//Anthony: needed since HardwareMap is initialized here, not in any other class
            robot.initTenserFlow();
            robot.initVuforia();

            waitForStart();

            handleAutoLoop();//Anthony: override just this
        }

        void handleAutoLoop() {
            if (opModeIsActive()) {
                switch (state) {

                    case 0:
                        minerals();
                        state++;
                        break;
                    case 1:
                        align();
                        state++;
                        break;
                    case 2:
                        depot();
                        state++;
                        break;
                    case 3:
                        crater();
                        state++;
                        break;
                    case 4:
                        requestOpModeStop();
                        break;
                }
            }
        }

//        public void unlatch() {
//            robot.lift.setPower(-1);
//            sleep(2000);
//            robot.lift.setPower(0);
//        }

        public void minerals() {
            robot.activateTenserFlow();
            //wait for start up
            sleep(500);
            robot.runTenserFlow();

            //distance between minerals: 14.5in, distance from robot to middle mineral: 23.5in
            //angle from robot to left or right minerals is arctan(14.5/23.5)=32deg
            //distance from robot to left or right minerals is sqrt(14.5^2+23.5^2)=27.6
            if (robot.mineralPostion == -1) {
                robot.turn(0.5, -31.7, 31.7);
                robot.drive(0.75, 27.6, 27.6);
            } else if(robot.mineralPostion == 1) {
                robot.turn(0.5, 31.7, -31.7);
                robot.drive(0.75, 27.6, 27.6);
            } else if (robot.check==true) {
                robot.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                robot.drive(0.75, 23.5, 23.5);
            }
        }

        public void align() {
            robot.pauseTenserFlow();
            robot.activateVuforia();
            robot.runVuforia();

            while(robot.targetVisible == false) {
                robot.runVuforia();
                robot.turn(0.1,-10,10);
                //TODO: turn robot left
            }

            if (robot.targetVisible == true) {
                if (robot.targetN == 'R' || robot.targetN == 'F') {
                    robot.drive(0.65,robot.y,robot.y);
                    robot.turn(0.75,-90,90);
                    //TODO: depot to the left, crater to the right

                } else if (robot.targetN == 'S' || robot.targetN == 'C') {
                    robot.drive(0.65,robot.x,robot.x);
                    robot.turn(0.75,90,-90);
                    //TODO: depot to the right, crater to the left
                }
            }
        }

        public void depot() {
            //TODO: drive to depot
            robot.drive(0.75,70,70);
            robot.dump.setPosition(1);
            sleep(500);
            robot.dump.setPosition(0);
        }

        public void crater() {
            robot.drive(0.75,-120.5,-120.5);
        }

}

