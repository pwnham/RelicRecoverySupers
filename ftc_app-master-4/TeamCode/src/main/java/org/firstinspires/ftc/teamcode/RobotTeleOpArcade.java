package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by moham on 1/13/18.
 */
@TeleOp
public class RobotTeleOpArcade extends OpMode {
    DcMotor leftBack, leftFront, intakeLeft, intakeRight, rightFront, rightBack;
    Servo clampLeft, clampRight, mandibleLeft, mandibleRight, jewelArm;
    CRServo conveyorLeft, conveyorRight, lift1, lift2;
    public double grabRightOut = .7;
    public double grabRightIn = .9;
    public double grabLeftOut = .85;
    public double grabLeftIn = .65;
    public double mandibleRightOut = .1;
    public double mandibleRightIn = .75;
    public double mandibleLeftOut = 1;
    public double mandibleLeftIn = .17;
    public double jewelArmUp = 1;
    public double jewelArmDown = .4;
    public double conveyorLeftStart;
    public double conveyorRightStart;
    public double driveMultiplier = 1;
    public double drivePower, driveTurn;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");

        conveyorRight = hardwareMap.crservo.get("conveyorRight");
        conveyorLeft = hardwareMap.crservo.get("conveyorLeft");
        lift1 = hardwareMap.crservo.get("liftTop");
        lift2 = hardwareMap.crservo.get("liftBottom");
        clampLeft = hardwareMap.servo.get("clampLeft");
        clampRight = hardwareMap.servo.get("clampRight");
        mandibleLeft = hardwareMap.servo.get("mandibleLeft");
        mandibleRight = hardwareMap.servo.get("mandibleRight");
        jewelArm = hardwareMap.servo.get("jewelArm");

//        conveyorLeftStart = conveyorLeft.getPosition();
//        conveyorRightStart = conveyorRight.getPosition();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);


        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        clampLeft.setPosition(grabLeftOut);
        clampRight.setPosition(grabRightOut);
        jewelArm.setPosition(jewelArmUp);
    }

    @Override
    public void loop() {
        drivePower = gamepad1.left_stick_y;
        driveTurn = gamepad1.right_stick_x;

        rightBack.setPower(drivePower - driveTurn);
        rightFront.setPower(drivePower - driveTurn);
        leftBack.setPower(drivePower + driveTurn);
        leftFront.setPower(drivePower + driveTurn);

        //intake
        //intake IN
        if (gamepad2.a) {
            intakeRight.setPower(1);
            intakeLeft.setPower(.7);
            conveyorLeft.setPower(-.5);
            conveyorRight.setPower(.5);
//            conveyorLeftStart -= .45;
//            conveyorRightStart += .45;
        } else if (gamepad2.b) { //INTAKE OUT
            intakeRight.setPower(-1);
            intakeLeft.setPower(-.7);
            conveyorLeft.setPower(.5);
            conveyorRight.setPower(-.5);
//            conveyorLeftStart += .45;
//            conveyorRightStart -= .45;
        } else {//INTAKE STOP
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
            conveyorLeft.setPower(0);
            conveyorRight.setPower(0);
//            conveyorLeftStart = .50;
//            conveyorRightStart = .50;
        }

//        conveyorLeft.setPosition(conveyorLeftStart);
//        conveyorRight.setPosition(conveyorRightStart);

        //clamp glyphs
        if (gamepad2.dpad_right) {
            clampLeft.setPosition(grabLeftIn);
            clampRight.setPosition(grabRightIn);
        } else if (gamepad2.dpad_left) {
            clampLeft.setPosition(grabLeftOut);
            clampRight.setPosition(grabRightOut);
        }

        //mandible
        if (gamepad2.x) {
            mandibleLeft.setPosition(mandibleLeftIn);
            mandibleRight.setPosition(mandibleRightIn);
        } else if (gamepad2.y) {
            mandibleLeft.setPosition(mandibleLeftOut);
            mandibleRight.setPosition(mandibleRightOut);
        }

        //lift
        if (gamepad2.dpad_down) {
            lift1.setPower(.3);
            lift2.setPower(.3);
        } else if (gamepad2.dpad_up) {
            lift1.setPower(-.3);
            lift2.setPower(-.3);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }


        //jewel arm
        if (gamepad1.dpad_down) {
            jewelArm.setPosition(jewelArmDown);
        } else if (gamepad1.dpad_up) {
            jewelArm.setPosition(jewelArmUp);
        }
    }
}
