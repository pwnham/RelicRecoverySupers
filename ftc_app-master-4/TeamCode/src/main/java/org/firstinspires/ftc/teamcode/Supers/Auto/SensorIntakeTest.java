package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by moham on 3/13/18.
 */
@TeleOp
public class SensorIntakeTest extends OpMode{

    public DcMotor intakeLeft, intakeRight;
    DistanceSensor distanceSensorLeft, distanceSensorRight;

    double leftDistance, rightDistance;

    String tilt;
    @Override
    public void init() {
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

    }

    @Override
    public void loop() {
        leftDistance = distanceSensorLeft.getDistance(DistanceUnit.INCH) - 2.5;
        rightDistance = distanceSensorRight.getDistance(DistanceUnit.INCH) - 2.5;

        telemetry.addData("Distance Left",
                String.format(Locale.US, "%.02f", leftDistance));
        telemetry.addData("Distance Right",
                String.format(Locale.US, "%.02f", rightDistance));

        double distanceRatio = leftDistance/rightDistance;

        telemetry.addData("Distance Ratio",
                String.format(Locale.US, "%.02f", distanceRatio));

        if (Math.abs(leftDistance-rightDistance) > 2) {
            if (leftDistance > rightDistance) {
                intakeLeft.setPower(.75);
                intakeRight.setPower(.25);
                tilt = "left";
            } else if (rightDistance > leftDistance) {
                intakeLeft.setPower(.25);
                intakeRight.setPower(.75);
                tilt = "right";
            }
        } else if (Math.abs(leftDistance-rightDistance) < 2){
            intakeLeft.setPower(.5);
            intakeRight.setPower(.5);
            tilt = "equal";
        } else {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            tilt = "none";
        }

        telemetry.addData("Tilt", tilt);


        //        telemetry.addData("leftDistance", leftDistance);
//        telemetry.addData("rightDistance", rightDistance);

//        double leftDistanceScaled = leftDistance/3;
//        double rightDistanceScaled = rightDistance/3;
//
//        if (leftDistanceScaled > 1 && rightDistanceScaled > 1) {
//            intakeLeft.setPower(0);
//            intakeRight.setPower(0);
//        } else if (leftDistanceScaled > rightDistanceScaled) {
//            intakeLeft.setPower(.25);
//            intakeRight.setPower(.5);
//        } else if (leftDistanceScaled < rightDistanceScaled) {
//            intakeLeft.setPower(.5);
//            intakeRight.setPower(.25);
//        } else {
//            intakeLeft.setPower(0);
//            intakeRight.setPower(0);
//        }


    }
}
