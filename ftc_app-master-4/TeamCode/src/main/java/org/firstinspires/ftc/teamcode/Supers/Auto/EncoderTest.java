package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 1/15/18.
 */
@Autonomous
public class EncoderTest extends Robot {

    private int leftBackTarget, leftFrontTarget, rightBackTarget, rightFrontTarget;

    private enum RobotState {//list states here
        Done, DriveForward
    }

    private RobotState robotState = RobotState.DriveForward;
    private boolean isleftFront = false, isleftBack = false, isrightFront = false, isrightBack = false;

    @Override
    public void init() {
        super.init();
        super.initAutonomous();
    }

    public void loop() {
        switch (robotState) {
            case DriveForward:
                if (resetPosition) {
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBackTarget = leftBack.getCurrentPosition() + 476;
                    leftFrontTarget = leftFront.getCurrentPosition() + 476;
                    rightBackTarget = rightBack.getCurrentPosition() + 476;
                    rightFrontTarget = rightFront.getCurrentPosition() + 476;

                    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (Math.abs(leftFrontTarget - leftFront.getCurrentPosition()) > 10) {
                    leftBack.setPower((leftBackTarget - leftBack.getCurrentPosition()) * PID_P);
                    leftFront.setPower((leftFrontTarget - leftFront.getCurrentPosition()) * PID_P);
                    rightBack.setPower((rightBackTarget - rightBack.getCurrentPosition()) * PID_P);
                    rightFront.setPower((rightFrontTarget - rightFront.getCurrentPosition()) * PID_P);
                    telemetry.addData("leftBack", leftBack.getCurrentPosition());
                    telemetry.addData("leftBackTarget", leftBackTarget);
                    telemetry.addData("leftFront", leftFront.getCurrentPosition());
                    telemetry.addData("leftFrontTarget", leftFrontTarget);
                    telemetry.addData("rightBack", rightBack.getCurrentPosition());
                    telemetry.addData("rightBackTarget", rightBackTarget);
                    telemetry.addData("rightFront", rightFront.getCurrentPosition());
                    telemetry.addData("rightFrontTarget", rightFrontTarget);
                } else {
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                }
                break;
            case Done:
                telemetry.addData("DONE", "DONE");
                break;
        }
    }

    public void stop() {
        super.stop();
    }
}