package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(72.5, 72.5, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action action = myBot.getDrive().actionBuilder(new Pose2d(-58.3, -45, Math.toRadians(235)))
                .setTangent(Math.toRadians(180))
//                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            this.setOuttakeSpeed(highVelocity);
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(15d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
//                )
//                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
//
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
                .strafeToSplineHeading(new Vector2d(36, -24), Math.toRadians(-90))
//                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
//                )
                .strafeToConstantHeading(new Vector2d(36, -58), new TranslationalVelConstraint(15))  // to intake 1st stack spot
//                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(highVelocity);
//                            blockUp();
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(-53d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
//                )
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot

//                .stopAndAdd(new SequentialAction(
//                        this.getAimAction(null, null, true),
//                        this.getLaunchAction(),
//                        this.getBlockDownAction()
//                ))


                .strafeToLinearHeading(new Vector2d(36, -64), Math.toRadians(0))
//                .afterDisp(0, this.getStartIntakeStopOuttakeAction()
//                )
                .strafeToConstantHeading(new Vector2d(62, -64))

                //                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(highVelocity);
//                            blockUp();
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(15d, HOOD_INITIAL_TARGET_POSITION_FAR_SIDE, false))
//                )
                .setTangent(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(59, -45))
                .strafeToLinearHeading(new Vector2d(53, -15), Math.toRadians(181)) //to launch spot
//                .stopAndAdd(new SequentialAction(
//                        this.getAimAction(null, null, true),
//                        this.getLaunchAction(),
//                        this.getBlockDownAction()
//                ))

                .strafeToConstantHeading(new Vector2d(36, -30))//park outside launch
                .build();
    myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}