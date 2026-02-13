

package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoBlueSideClose2StackOpenGate extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, -45, Math.toRadians(235));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .afterDisp(0, new ParallelAction(
                        this.getStopIntakeStartOuttakeAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(null, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToSplineHeading(new Vector2d(12, -18), Math.toRadians(-90))  //turn before intake

                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(14, -54), new TranslationalVelConstraint(100))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(
                                this.getBlockUpAction(),
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()

                ))
                .strafeToLinearHeading(new Vector2d(14, -36), Math.toRadians(-90))
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToLinearHeading(new Vector2d(14, -56), Math.toRadians(-115)) // open gate
                .strafeToConstantHeading(new Vector2d(16, -58)) // move away from gate

                .stopAndAdd(new SleepAction(2))

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 3rd time
                .afterDisp(0, new ParallelAction(
                                this.getBlockUpAction(),
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-15d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToConstantHeading(new Vector2d(12, -36))
                .strafeToSplineHeading(new Vector2d(10, -56), Math.toRadians(-115))
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-15d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))


                .strafeToConstantHeading(new Vector2d(12, -36))

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))

                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
                .build();
    }




    @Override
    protected double getTurretDegreeOffset() {
        return 0;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }


}