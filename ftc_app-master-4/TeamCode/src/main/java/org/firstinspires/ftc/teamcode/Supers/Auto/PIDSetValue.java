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
public class PIDSetValue extends OpMode {

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
//        PIDCoefficients pidOrig = leftBack.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

//        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
//        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//        // re-read coefficients and verify change.
//        PIDCoefficients pidModified = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        PIDCoefficients pidNewBack = new PIDCoefficients(10, 0, 0);
        PIDCoefficients pidNewFront = new PIDCoefficients(10, 0, 0);
        leftBack.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));
        leftFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));
        rightBack.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));
        rightFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));

    }
}
