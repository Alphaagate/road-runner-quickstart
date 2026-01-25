

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoRedSideClose1Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, 44.5, Math.toRadians(-235));
    }
    @Override
    protected Action getPathAction() {

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(getInitialPose())
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-5d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(90))   //change heading
                .afterDisp(0, new SequentialAction(
                        telemetryPacket -> {
                            this.reverseOuttake();
                            return false;
                        }, this.getIntakeAction()

                ))
                .strafeToConstantHeading(new Vector2d(-12, 48))                     //to intake
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-40d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(-30d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, true),
                        this.getLaunchAction()
                ));

        return this.strafeToOpenGate(actionBuilder, FieldSide.RED)      //open the gate if shouldOpenGate == true
                .strafeToConstantHeading(new Vector2d(-12, 35))   //park outside launch
                .build();
    }


    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

}