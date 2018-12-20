package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robo extends LinearOpMode {
    public DcMotor left;
    public DcMotor right;
    public DcMotor leftb;
    public DcMotor rightb;
    public DcMotor lift;
    public DcMotor lift2;
    //public DcMotor intake;
    public Servo dump;
    public Servo hook;

    @Override
    public void runOpMode() throws InterruptedException {
        if(hardwareMap == null) {
            OpMode active = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).getActiveOpMode();
            super.hardwareMap = active.hardwareMap;
            super.telemetry = active.telemetry;
        }

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        leftb = hardwareMap.get(DcMotor.class, "leftb");
        rightb = hardwareMap.get(DcMotor.class, "rightb");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift = hardwareMap.get(DcMotor.class, "lift");
        //   intake = hardwareMap.get(DcMotor.class, "intake");
        dump = hardwareMap.get(Servo.class, "dump");
        hook = hardwareMap.get(Servo.class, "hook");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //  intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.FORWARD);
        leftb.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        // intake.setDirection(DcMotor.Direction.REVERSE);
    }
//    static final double COUNTS_PER_MOTOR_REV = 537.6;
//    static final double COUNTS_PER_MOTOR_REVBACK = 1680;
//    static final double DRIVE_GEAR_REDUCTION = 0.45;
//    static final double WHEEL_DIAMETER_INCHES = 4.0;
//
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double COUNTS_PER_INCHBACK = (COUNTS_PER_MOTOR_REVBACK * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    static final double ROBOT_RADIUS = 9.0;
//
//    static final double INCHES_PER_DEGREE = (Math.PI / 180) * ROBOT_RADIUS;
//
//    public void drive(double speed, double inches) {
//        encoders(speed, 0, 0, inches);
//    }
//
//    public void sidestep(double speed, double inches) {
//        encoders(0, speed, 0, inches);
//    }
//
//    public void turn(double speed, double degrees) {
//        encoders(0, 0, speed, degrees * INCHES_PER_DEGREE);
//    }
//
//    public void clearEncoders() {
//        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    private void encoders(double forward, double sideways, double turn, double inches) {
//        int newLeftTarget;
//        int newRightTarget;
//        int newLeftBTarget;
//        int newRightBTarget;
//
//        if (opModeIsActive()) {
//            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = left.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newLeftBTarget = leftb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newRightTarget = right.getCurrentPosition() + (int) (inches * COUNTS_PER_INCHBACK);
//            newRightBTarget = rightb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCHBACK);
//
//            left.setTargetPosition(newLeftTarget);
//            right.setTargetPosition(newRightTarget);
//            leftb.setTargetPosition(newLeftBTarget);
//            rightb.setTargetPosition(newRightBTarget);
//            // Turn On RUN_TO_POSITION
//            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // reset the timeout time and start motion.
//            setPowers(forward, sideways, turn);
//
//            while (opModeIsActive() && (left.isBusy() && right.isBusy() && leftb.isBusy() && rightb.isBusy())) {
//                // Display it for the driver.
//                telemetry.addData("RoboPath1", "Running to %7d :%7d %7d :%7d", newLeftTarget, newRightTarget, newLeftBTarget, newRightBTarget);
//                telemetry.addData("RoboPath2", "Running at %7d :%7d %7d :%7d", left.getCurrentPosition(), right.getCurrentPosition(), leftb.getCurrentPosition(), rightb.getCurrentPosition());
//                telemetry.update();
//            }
//            setPowers(0, 0, 0);
//
//            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//    }
//
//    public void setPowers(double forward, double sideways, double turn) {
//        //converts xy coordinates to polar coordinates
//        double r = Math.hypot(sideways, forward);
//        //calculates the current heading of the robot
//        double robotAngle = Math.atan2(forward, sideways) - Math.PI / 4;
//        double v1 = r * Math.cos(robotAngle) + turn;
//        double v2 = r * Math.sin(robotAngle) - turn;
//        double v3 = r * Math.sin(robotAngle) + turn;
//        double v4 = r * Math.cos(robotAngle) - turn;
//        v1 = Range.clip(v1, -1, 1);
//        v2 = Range.clip(v2, -1, 1);
//        v3 = Range.clip(v3, -1, 1);
//        v4 = Range.clip(v4, -1, 1);
//
//        left.setPower(v1);
//        right.setPower(v2);
//        leftb.setPower(v3);
//        rightb.setPower(v4);
//    }
}
