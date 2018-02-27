package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by moham on 1/23/18.
 */
@TeleOp
public class PIDTune extends OpMode {

    public DcMotor slideLeft, slideRight, intakeRight, intakeLeft;
    public Servo flipLeft, flipRight;
    public DcMotorEx leftBack, leftFront, rightFront, rightBack;

    public BNO055IMU imu;

    public Orientation angles;
    Acceleration gravity;

    public BNO055IMU.Parameters imuparameters;

    public boolean resetPosition = true;
    public static final int ENCODER_TICKS_PER_CM = 17 ; //  538/(10 * Math.PI) = 17.1337579618
    public int targetTicks;
    public double PID_P = .003;
    public double flipRightUp = .25;
    public double flipRightDown = .8;
    public double flipLeftUp = .75;
    public double flipleftDown = .2;
    private double leftBackKP, leftBackKI, leftBackKD, leftFrontKP, leftFrontKI, leftFrontKD, rightBackKP, rightBackKI, rightBackKD, rightFrontKP, rightFrontKI, rightFrontKD;

    @Override
    public void init() {
        leftBack = (DcMotorEx)hardwareMap.dcMotor.get("leftBack");
        rightBack = (DcMotorEx)hardwareMap.dcMotor.get("rightBack");
        rightFront = (DcMotorEx)hardwareMap.dcMotor.get("rightFront");
        leftFront = (DcMotorEx)hardwareMap.dcMotor.get("leftFront");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        flipLeft = hardwareMap.servo.get("flipLeft");
        flipRight = hardwareMap.servo.get("flipRight");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipLeft.setPosition(flipleftDown);
        flipRight.setPosition(flipRightDown);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients leftBackPID = leftBack.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients leftFrontPID = leftFront.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients rightBackPID = rightBack.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients rightFrontPID = rightFront.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);


//        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
//        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//        // re-read coefficients and verify change.
//        PIDCoefficients pidModified = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackKP = leftBackPID.p;
        leftBackKI = leftBackPID.i;
        leftBackKD = leftBackPID.d;
        leftFrontKP = leftFrontPID.p;
        leftFrontKI = leftFrontPID.i;
        leftFrontKD = leftFrontPID.d;
        rightBackKP = rightBackPID.p;
        rightBackKI = rightBackPID.i;
        rightBackKD = rightBackPID.d;
        rightFrontKP = rightFrontPID.p;
        rightFrontKI = rightFrontPID.i;
        rightFrontKD = rightFrontPID.d;


    }

    @Override
    public void loop() {
        telemetry.addData("leftBackKP", leftBackKP);
        telemetry.addData("leftBackKI", leftBackKI);
        telemetry.addData("leftBackKD", leftBackKD);

        telemetry.addData("leftFrontKP", leftFrontKP);
        telemetry.addData("leftFrontKI", leftFrontKI);
        telemetry.addData("leftFrontKD", leftFrontKD);

        telemetry.addData("rightBackKP", rightBackKP);
        telemetry.addData("rightBackKI", rightBackKI);
        telemetry.addData("rightBackKD", rightBackKD);

        telemetry.addData("rightFrontKP", rightFrontKP);
        telemetry.addData("rightFrontKI", rightFrontKI);
        telemetry.addData("rightFrontKD", rightFrontKD);


//        if (gamepad1.a && gamepad1.dpad_up) {
//            kP += .1;
//        } else if (gamepad1.a && gamepad1.dpad_down) {
//            kP -= .1;
//        } else if (gamepad1.a && gamepad1.dpad_right) {
//            kP += .01;
//        } else if (gamepad1.a && gamepad1.dpad_left) {
//            kP -= .01;
//        }
//
//        if (gamepad1.b && gamepad1.dpad_up) {
//            kI += .1;
//        } else if (gamepad1.b && gamepad1.dpad_down) {
//            kI -= .1;
//        } else if (gamepad1.b && gamepad1.dpad_right) {
//            kI += .01;
//        } else if (gamepad1.b && gamepad1.dpad_left) {
//            kI -= .01;
//        }
//
//        if (gamepad1.x && gamepad1.dpad_up) {
//            kD += .1;
//        } else if (gamepad1.x && gamepad1.dpad_down) {
//            kD -= .1;
//        } else if (gamepad1.x && gamepad1.dpad_right) {
//            kD += .01;
//        } else if (gamepad1.x && gamepad1.dpad_left) {
//            kD -= .01;
//        }

//        PIDCoefficients pidNew = new PIDCoefficients(kP, kI, kD);
//        leftBack.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
//        leftFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
//        rightBack.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
//        rightFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

    }
}
