package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 1/19/18.
 */
@TeleOp
public class IMUTurnTest extends Robot {

    private enum RobotState {//list states here
        Done, TurnRight
    }

    private RobotState robotState = RobotState.TurnRight;

    private boolean gyroInitialized = false;

    @Override
    public void init() {
        super.init();
        super.initAutonomous();
    }

    @Override
    public void loop() {
        if (!gyroInitialized) {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            gyroInitialized = true;
        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).toAngleUnit(AngleUnit.DEGREES);

        switch (robotState) {
            case TurnRight:
                if (resetPosition) {
                    imu.initialize(imuparameters);
                    resetPosition = false;
                    try {
                        wait(1000);
                    } catch (Exception e) {
                    }
                }
                if (angles.firstAngle - 90 < 2) {
                    leftBack.setPower(-.15);
                    leftFront.setPower(-.15);
                    rightBack.setPower(.15);
                    rightFront.setPower(.15);
                } else if (angles.firstAngle - 90 > 2) {
                    leftBack.setPower(.15);
                    leftFront.setPower(.15);
                    rightBack.setPower(-.15);
                    rightFront.setPower(-.15);
                } else {
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    robotState = RobotState.Done;
                }
                break;
            case Done:
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                break;
        }
        telemetry.addData("heading", angles.firstAngle);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
