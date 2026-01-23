

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoBlueSideClose1Stack extends AbstractFullAuto {

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, -45, Math.toRadians(235));
    }

    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                    this.setOuttakeSpeed(lowVelocity);
                    return false;
                }, this.getAimAction(-5d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))) // set the initial turret degree and hood position
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                //Note: I think following is problematic as the stopAndAdd already stop the trajectory segment,
                //      so afterDisp has no last segment to attach to.
//                .afterDisp(0, telemetryPacket -> {
//                    this.reverseOuttake();
//                    return false;
//                })
                .strafeToSplineHeading(new Vector2d(-12, -24), Math.toRadians(-90))   //change heading
                .afterDisp(0, new SequentialAction(
                        telemetryPacket -> {
                            this.reverseOuttake();
                            return false;
                        }, this.getIntakeAction()

                ))
                .strafeToConstantHeading(new Vector2d(-12, -54))  //to intake
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            return false;
                        },
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-40d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(-30d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, true),
                        this.getLaunchAction()
                ))
                .strafeToConstantHeading(new Vector2d(-12, -35))   //park outside launch
                .build();
    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }


}