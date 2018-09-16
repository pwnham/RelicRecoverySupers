package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 6/18/18.
 */
@Autonomous
public class DrivePID extends Robot {
    boolean resetPositon = true;

    @Override
    public void init() {
        super.init();
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (resetPosition) {


            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setTargetPosition(1455);
            leftFront.setTargetPosition(1455);
            rightBack.setTargetPosition(1455);
            rightFront.setTargetPosition(1455);


            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBack.setPower(.5);
            leftFront.setPower(.5);
            rightBack.setPower(.5);
            rightFront.setPower(.5);
            resetPosition = false;
        }
    }
}
