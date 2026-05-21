

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
//@Autonomous(group = "Autonomous")
public class BlueClose3Stack extends AbstractFullAuto {
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
                .strafeToSplineHeading(new Vector2d(-12, -18), Math.toRadians(-85))  //turn before intake

                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(-12, -52), new TranslationalVelConstraint(100))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToSplineHeading(new Vector2d(12, -18), Math.toRadians(-82))  //turn before intake

                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(14, -57), new TranslationalVelConstraint(100))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToSplineHeading(new Vector2d(14, -18), Math.toRadians(-85))  //turn before intake

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()

                ))


                .strafeToSplineHeading(new Vector2d(36, -18), Math.toRadians(-85))  //turn before intake

                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(36, -64), new TranslationalVelConstraint(100))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))

                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
                .build();
    }




    @Override
    protected double getTurretDegreeOffset() {
        return 5;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

    @Override
    protected double getCloseOrFar() {
        return 1;
    }
}