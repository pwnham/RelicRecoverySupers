package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by moham on 11/15/17.
 */

@Autonomous
public class MichaelHouseAutoTest extends OpMode{
    DcMotor rightBack, rightFront, leftBack, leftFront;
    private ElapsedTime runtime = new ElapsedTime();
    boolean initialized;

    enum AutoState{forward, backward, right, left, stop}
    AutoState state = AutoState.forward;

    @Override
    public void init() {
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);

    }

    @Override
    public void loop() {
        telemetry.addData("Robot State: ", state);

        rightBack.setPower(1);
        rightFront.setPower(1);
        leftBack.setPower(1);
        leftFront.setPower(1);

//        switch (state) {
//            case forward:
//                if (!initialized) {
//                    runtime.reset();
//                    initialized = true;
//                }
//                if (runtime.seconds() < 2) {
//                    rightBack.setPower(1);
//                    rightFront.setPower(1);
//                    leftBack.setPower(1);
//                    rightFront.setPower(1);
//                } else {
//                    state = AutoState.backward;
//                    initialized = false;
//                }
//                break;
//            case backward:
//                if (!initialized) {
//                    runtime.reset();
//                    initialized = true;
//                }
//                if (runtime.seconds() < 2) {
//                    rightBack.setPower(-1);
//                    rightFront.setPower(-1);
//                    leftBack.setPower(-1);
//                    rightFront.setPower(-1);
//                } else {
//                    state = AutoState.right;
//                    initialized = false;
//                }
//                break;
//            case right:
//                if (!initialized) {
//                    runtime.reset();
//                    initialized = true;
//                }
//                if (runtime.seconds() < 2) {
//                    rightBack.setPower(1);
//                    rightFront.setPower(1);
//                    leftBack.setPower(-1);
//                    rightFront.setPower(-1);
//                } else {
//                    state = AutoState.left;
//                    initialized = false;
//                }
//                break;
//            case left:
//                if (!initialized) {
//                    runtime.reset();
//                    initialized = true;
//                }
//                if (runtime.seconds() < 2) {
//                    rightBack.setPower(-1);
//                    rightFront.setPower(-1);
//                    leftBack.setPower(1);
//                    rightFront.setPower(1);
//                } else {
//                    state = AutoState.stop;
//                    initialized = false;
//                }
//                break;
//            case stop:
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//                leftBack.setPower(0);
//                rightFront.setPower(0);
//        }

    }
}
