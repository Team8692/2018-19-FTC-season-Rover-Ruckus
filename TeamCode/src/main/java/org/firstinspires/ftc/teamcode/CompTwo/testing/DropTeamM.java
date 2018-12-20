package org.firstinspires.ftc.teamcode.CompTwo.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class DropTeamM extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    initHardware();
            //TODO: drive to depot
            drive(0.75,70,70);
            dump.setPosition(1);
            sleep(500);
            dump.setPosition(0);

    }
    public DcMotor left;
    public DcMotor right;
    public DcMotor leftb;
    public DcMotor rightb;
    public DcMotor intake;
    //public DcMotor intake2;
    public DcMotor lift;
    public Servo dump; //to radians 2. get arc length (does not need to multiply by 2, since robot is symmetrical) 3. drive calculated arc length

    public void initHardware(){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        leftb = hardwareMap.get(DcMotor.class, "leftb");
        rightb = hardwareMap.get(DcMotor.class, "rightb");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake2 = hardwareMap.get(DcMotor.class, "intake2");
        lift = hardwareMap.get(DcMotor.class, "lift");
        dump = hardwareMap.get(Servo.class, "dump");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    static final double COUNTS_PER_MOTOR_REV = 1680;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double ROBOT_RADIUS = 9.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE = (Math.PI / 180) * ROBOT_RADIUS * COUNTS_PER_INCH;
    public void drive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            leftb.setPower(Math.abs(speed));
            rightb.setPower(Math.abs(speed));

            while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("RoboPath1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("RoboPath2", "Running at %7d :%7d", left.getCurrentPosition(),
                        right.getCurrentPosition());
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
        }
    }

    //Anthony: added turning code using arc length
    public void turn(double speed, double leftDegrees, double rightDegrees) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) (leftDegrees * COUNTS_PER_DEGREE);
            newRightTarget = right.getCurrentPosition() + (int) (rightDegrees * COUNTS_PER_DEGREE);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            leftb.setPower(Math.abs(speed));
            rightb.setPower(Math.abs(speed));

            while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("RoboPath1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("RoboPath2", "Running at %7d :%7d", left.getCurrentPosition(),
                        right.getCurrentPosition());
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
        }
    }
}

