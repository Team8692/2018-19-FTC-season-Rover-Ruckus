package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RentonCV;
import org.firstinspires.ftc.teamcode.Robo;

@Autonomous(name = "Emergency Depot Only!!")
public class test extends Robo {
    RentonCV vision = new RentonCV();
    int goldp = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //vision.runOpMode();
        while(opModeIsActive()) {
            hook.setPosition(0);
//
//            sleep(2000);
//
//            lift2.setPower(-.2);
//            sleep(2000);
//
//            lift.setPower(-.2);
//            sleep(2000);
//            lift.setPower(0);
//
//            lift2.setPower(0);
//
//            while(vision.found==false) {
//                left.setPower(-0.1);
//                right.setPower(0.1);
//                leftb.setPower(-0.3);
//                rightb.setPower(0.3);
//            }
//            left.setPower(0);
//            right.setPower(0);
//            leftb.setPower(0);
//            rightb.setPower(0);
//
//            //vision.updateCam();
//            sleep(2000);
//
//            if(vision.found==true && vision.targetVisible==false){
//                if (vision.goldPos<280){
//
//                    while(vision.align==false){
//                        left.setPower(0.1);
//                        right.setPower(-0.1);
//                        rightb.setPower(-0.3);
//                        leftb.setPower(0.3);
//                    }
//                }else if (vision.goldPos>360){
//                    while(vision.align==false){
//                        left.setPower(-0.1);
//                        right.setPower(0.1);
//                        rightb.setPower(0.3);
//                        leftb.setPower(-0.3);
//                    }
//                }
//
//            }
           //// vision.vuforia.stop();

//            left.setPower(1);
//            right.setPower(1);
//            rightb.setPower(1);
//            leftb.setPower(1);
//            sleep(2000);
//            left.setPower(0);
//            right.setPower(0);
//            rightb.setPower(0);
//            leftb.setPower(0);
//
//            dump.setPosition(1);
//            sleep(1000);
//            dump.setPosition(0);

        }
    }

}
