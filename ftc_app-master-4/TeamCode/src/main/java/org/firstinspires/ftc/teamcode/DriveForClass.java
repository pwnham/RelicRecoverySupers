package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 6/18/18.
 */
@Autonomous
public class DriveForClass extends Robot {
    boolean initialized = false;
    long time;
    int state = 0;

    @Override
    public void init() {
        super.init();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (!initialized) {
            time = System.currentTimeMillis();
            initialized = true;
        }

        switch (state) {
            case 0: //drive forward
                rightBack.setPower(1);
                rightFront.setPower(1);
                leftBack.setPower(1);
                leftFront.setPower(1);
                break;
            case 1://turn left
                rightBack.setPower(.3);
                rightFront.setPower(.3);
                leftBack.setPower(-.3);
                leftFront.setPower(-.3);
                break;
            case 2://park and move jewel arm down
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);
                jewelArm.setPosition(jewelArmDown);
                break;
            case 3://park
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);
                break;
        }


        //STATE SWITCHING ON TIME
        if (System.currentTimeMillis() - time <=750) {
            state = 0;
        } else if ((System.currentTimeMillis() - time > 750) && (System.currentTimeMillis() - time <= 2000)){
//            state = 1;
            state = 3;
        } else if ((System.currentTimeMillis() - time > 2000) && (System.currentTimeMillis() - time <= 3000)){
            state = 3;
        }

    }
}
