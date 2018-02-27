package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByTime_Linear;

/**
 * Created by HomeFolder on 11/11/17.
 */

@Autonomous
public class ScrimRedR extends ScrimRobot {

    private ElapsedTime runtime = new ElapsedTime();
    private ScrimRobot robot = new ScrimRobot();

    enum AutoState{DriveForward, Right, DriveMore, Raise, VerticalBlocks, Release, Stop}
    AutoState state = AutoState.DriveForward;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Robot State: ", state);

        switch(state) {
            case DriveForward:
                if (drive(driveState.Forwards, .5, 2)) {
                    state = AutoState.Right;
                    break;
                }

            case Right:
                if (drive(driveState.TurnRight, .5, 2)) {
                    state = AutoState.DriveMore;
                    break;
                }

            case DriveMore:
                if (drive(driveState.Forwards, .5, .7)) {
                    state = AutoState.Raise;
                    break;
                }

            case Raise:
                if (liftandRelease(driveState.LiftSlide, .8, 0, 1.5)) {
                    state = AutoState.VerticalBlocks;
                    break;
                }

            case VerticalBlocks:
                if (liftandRelease(driveState.MoveBlocks, .8, .5, 1.5)) {
                    state = AutoState.Release;
                    break;
                }

            case Release:
                if (robot.liftandRelease(driveState.Release, 0, 0, 0)) {
                    state = AutoState.Stop;
                    break;
                }

            case Stop:
                robot.stop();

        }






    }

}