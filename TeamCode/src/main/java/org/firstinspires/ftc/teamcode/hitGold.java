//package org.firstinspires.ftc.teamcode.SAFE;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//
//@Autonomous(name = "testing: hit gold mineral")
//public class hitGold extends LinearOpMode {
//
//    org.firstinspires.ftc.teamcode.SAFE.Robot robot = new org.firstinspires.ftc.teamcode.SAFE.Robot();
//    RentonCV vision = new RentonCV();
//    static int goldLoc = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.runOpMode();
//        vision.runOpMode();
//
//        while(!isStarted()) {
//            vision.updateCam();
//        }
//
//        vision.updateCam();
//        if(!isStopRequested()) {
//            if (vision.found && !vision.align && vision.targetVisible==false && opModeIsActive()) {
//                if (vision.goldPos < 320) {
//                    robot.sidestep(-0.5, 14.5);
//                    goldLoc = -1;
//                } else if (vision.goldPos > 320) {
//                    robot.sidestep(0.5, 14.5);
//                    goldLoc = 1;
//                }
//                robot.drive(0.75, 23.5);
//            }
//        }
//
//        vision.vuforia.stop();
//    }
//}
