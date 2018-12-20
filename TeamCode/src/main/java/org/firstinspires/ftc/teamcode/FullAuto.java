package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Full Auto")
public class FullAuto extends Robo {

    RentonCV vision = new RentonCV();
    static int goldLoc = 0;
    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        vision.runOpMode();

       // dump.setPosition(.2);
        hook.setPosition(1);


        if(!isStopRequested() && opModeIsActive()) {
            hook.setPosition(0);

            sleep(2000);

            lift2.setPower(-.2);
            sleep(2000);

            lift.setPower(-.2);
            sleep(2000);
            lift.setPower(0);
////////////////////////////////////////////////////////////////////////////////
            lift2.setPower(0);

            spin();

           minerals();
           align();
           depot();
           crater();



        }
    }
    public void RunCamera(){
        while(!isStopRequested()) {
            vision.updateCam();
        }
        vision.vuforia.stop();
    }

    public void unlatch(){
        hook.setPosition(1);
        sleep(2000);
        lift.setPower(.3);
        lift2.setPower(.3);
        sleep(1000);

        lift.setPower(0);
        lift2.setPower(0);

        dump.setPosition(0);
    }

    public void spin(){
        left.setPower(-0.1);
        right.setPower(0.1);
        leftb.setPower(-0.3);
        rightb.setPower(0.3);
    }

    public void minerals(){

        if(vision.align = true && vision.found==true){
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
            sleep(1000);
            left.setPower(0.5);
            right.setPower(0.5);
            leftb.setPower(1);
            rightb.setPower(1);
            sleep(2500);
        }

    }

    public void align(){
        drive(-0.5,10);

        turn(-0.5,90);

        drive(0.5,40);

        turn(-.5,90);

        drive(.5,72);

    }

    public void depot(){
        dump.setPosition(1);
        sleep(500);
        dump.setPosition(0);
    }

    public void crater(){
        drive(-0.5, -96);
    }

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double COUNTS_PER_MOTOR_REVBACK = 1680;
    static final double DRIVE_GEAR_REDUCTION = 0.45;
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCHBACK = (COUNTS_PER_MOTOR_REVBACK * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double ROBOT_RADIUS = 9.0;

    static final double INCHES_PER_DEGREE = (Math.PI / 180) * ROBOT_RADIUS;

    public void drive(double speed, double inches) {
        encoders(speed, 0, 0, inches);
    }

    public void sidestep(double speed, double inches) {
        encoders(0, speed, 0, inches);
    }

    public void turn(double speed, double degrees) {
        encoders(0, 0, speed, degrees * INCHES_PER_DEGREE);
    }

    public void clearEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void encoders(double forward, double sideways, double turn, double inches) {
        int newLeftTarget;
        int newRightTarget;
        //int newLeftBTarget;
        //int newRightBTarget;

        if (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
          //  newLeftBTarget = leftb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int) (inches * COUNTS_PER_INCHBACK);
//            newRightBTarget = rightb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCHBACK);

            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            //leftb.setTargetPosition(newLeftBTarget);
           // rightb.setTargetPosition(newRightBTarget);
            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            setPowers(forward, sideways, turn);

            while (opModeIsActive() && (left.isBusy() && right.isBusy() && leftb.isBusy() && rightb.isBusy())) {
                // Display it for the driver.
               // telemetry.addData("RoboPath1", "Running to %7d :%7d %7d :%7d", newLeftTarget, newRightTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("RoboPath2", "Running at %7d :%7d %7d :%7d", left.getCurrentPosition(), right.getCurrentPosition(), leftb.getCurrentPosition(), rightb.getCurrentPosition());
                telemetry.update();
            }
            setPowers(0, 0, 0);

            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPowers(double forward, double sideways, double turn) {
        //converts xy coordinates to polar coordinates
        double r = Math.hypot(sideways, forward);
        //calculates the current heading of the robot
        double robotAngle = Math.atan2(forward, sideways) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;
        v1 = Range.clip(v1, -1, 1);
        v2 = Range.clip(v2, -1, 1);
        v3 = Range.clip(v3, -1, 1);
        v4 = Range.clip(v4, -1, 1);

        left.setPower(v1);
        right.setPower(v2);
        leftb.setPower(v3);
        rightb.setPower(v4);
    }
}
