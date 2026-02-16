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
public class RedFarLoadingZone extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }


    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, 15, Math.toRadians(180));
    }
    @Override
    protected Action getPathAction() {
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))

                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
//
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
                .strafeToSplineHeading(new Vector2d(36, 24), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(telemetryPacket -> {
                            this.setOuttakeSpeed(0);
                            return false;
                        }, this.getIntakeAction())
                )
                .strafeToConstantHeading(new Vector2d(36, 58), new TranslationalVelConstraint(15))  // to intake 1st stack spot
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(highVelocity);
                            blockUp();
                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(53d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(53, 15)) //to launch spot

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


                .strafeToLinearHeading(new Vector2d(36, 64), Math.toRadians(0))
                //                .afterDisp(0, new SequentialAction(telemetryPacket -> {
//                            this.setOuttakeSpeed(0);
//                            return false;
//                        }, this.getIntakeAction())
//                )
                .strafeToConstantHeading(new Vector2d(62, 64))

                //                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(highVelocity);
//                            blockUp();
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(-15d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
//                )
                .setTangent(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(59, 45))
                .strafeToLinearHeading(new Vector2d(53, 15), Math.toRadians(179)) //to launch spot
//                .stopAndAdd(new SequentialAction(
//                        //Further adjust the aiming before launching
//                        this.getAimAction(null, null, true),
//                        this.getLaunchAction(),
//                        telemetryPacket -> {
//                            this.sleep(300);
//                            this.blockDown();
//                            return false;
//                        }
//                ))

                .strafeToConstantHeading(new Vector2d(36, 30))//park outside launch
                .build();

    }

    @Override
    protected double getTurretDegreeOffset() {
        return -4d;
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