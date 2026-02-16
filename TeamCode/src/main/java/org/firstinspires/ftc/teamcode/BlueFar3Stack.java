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
@Autonomous(group = "Autonomous")
public class BlueFar3Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, -15, Math.toRadians(180));
    }
    @Override
    protected Action getPathAction() {
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(15d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))
                .strafeToSplineHeading(new Vector2d(36, -24), Math.toRadians(-90))
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(36, -58), new TranslationalVelConstraint(15))  // to intake 1st stack spot
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-53d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))


                .strafeToSplineHeading(new Vector2d(12, -24), Math.toRadians(-90))
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )

                .strafeToConstantHeading(new Vector2d(12, -58), new TranslationalVelConstraint(15))  // to intake 2nd stack spot
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-53d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        this.getBlockDownAction()
                ))

                .strafeToSplineHeading(new Vector2d(-14, -24), Math.toRadians(-90))
                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
                )
                .strafeToConstantHeading(new Vector2d(-14, -48), new TranslationalVelConstraint(20))  // to intake 3rd stack spot //, new TranslationalVelConstraint(30.0)
                .afterDisp(0, new ParallelAction(
                        this.getBlockUpAction(),
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-53d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                .strafeToConstantHeading(new Vector2d(36, -30))//park outside launch
                .build();
    }

    @Override
    protected double getTurretDegreeOffset() {
        return 0;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_FAR, 0, 0, NEW_F_FAR);
    }

    @Override
    protected double getCloseOrFar() {
        return 2;
    }
}