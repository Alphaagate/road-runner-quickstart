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
public class FullAutoBlueSideFar1Stack extends AbstractFullAuto {
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
        double xOffset = 0;
        double yOffset = 0;
        double degreeOffset = 0;

        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            this.setOuttakeSpeed(highVelocity);
                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(15d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
                .stopAndAdd(new SequentialAction(
                        //Further adjust the aiming before launching
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        telemetryPacket -> {
                            this.sleep(300);
                            this.blockDown();
                            return false;
                        }


                ))
                .strafeToSplineHeading(new Vector2d(36 + xOffset, -24), Math.toRadians(-90 + degreeOffset))
                .afterDisp(0, new SequentialAction(telemetryPacket -> {
                            this.intakeMotor.setPower(1);
                            this.setOuttakeSpeed(0);
                            return false;
                        })
                )
                .strafeToConstantHeading(new Vector2d(36 + xOffset, -58), new TranslationalVelConstraint(20))  // to intake spot
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(highVelocity);
                            this.blockUp();

                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-47d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53 + xOffset, -15 + yOffset )) //to launch spot
                .stopAndAdd(new SequentialAction(
                        //Further adjust the aiming before launching
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                .strafeToConstantHeading(new Vector2d(36, -30))//park outside launch
                .build();

    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_FAR, 0, 0, NEW_F_FAR);
    }

}