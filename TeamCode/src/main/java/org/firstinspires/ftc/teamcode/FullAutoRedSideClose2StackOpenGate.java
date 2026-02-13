

package org.firstinspires.ftc.teamcode;


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
public class FullAutoRedSideClose2StackOpenGate extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, 45, Math.toRadians(-235));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .afterDisp(0, new ParallelAction(
                        this.getStopIntakeStartOuttakeAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(4d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, 12))//to launch spot
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToSplineHeading(new Vector2d(14, 18), Math.toRadians(85))  //turn before intake

                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(14, 56), new TranslationalVelConstraint(30))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .setTangent(180)
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot 1st time
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()

                ))
                .strafeToConstantHeading(new Vector2d(6, 36))
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToLinearHeading(new Vector2d(6, 52), Math.toRadians(85)) // open gate


                .strafeToLinearHeading(new Vector2d(18, 60), Math.toRadians(145)) // move away from gate

                .stopAndAdd(new SleepAction(1.5))
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(85))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(85))   //change heading
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(-12, 52), new TranslationalVelConstraint(20))  //to intake 1st stack



                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )

                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))

                .strafeToConstantHeading(new Vector2d(-12, 35))                     //park outside launch
                .build();
    }

    @Override
    protected double getTurretDegreeOffset() {
        return -2d;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

}