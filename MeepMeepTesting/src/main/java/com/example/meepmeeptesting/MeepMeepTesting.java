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
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action action = myBot.getDrive().actionBuilder(new Pose2d(63, -15, Math.toRadians(180)))
//                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(lowVelocity);
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(-5d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
//                )
                .strafeToConstantHeading(new Vector2d(-12, 12))//to launch spot
//                .stopAndAdd(new SequentialAction(
//                        this.getAimAction(-30d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, true),
//                        this.getLaunchAction()
//                ))
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(-90))   //change heading
//                .afterDisp(0, new SequentialAction(
//                        telemetryPacket -> {
//                            this.reverseOuttake();
//                            return false;
//                        }, this.getIntakeAction()
//
//                ))
                .strafeToConstantHeading(new Vector2d(-12, 48))                     //to intake
//                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(lowVelocity);
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(-40d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
//                )
                .strafeToConstantHeading(new Vector2d(-12, 12))//to launch spot
//                .stopAndAdd(new SequentialAction(
//                        this.getAimAction(-30d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, true),
//                        this.getLaunchAction()
//                ))

                .strafeToSplineHeading(new Vector2d(12, 24), Math.toRadians(-90))  //to launch spot
//                .afterDisp(0, new ParallelAction(telemetryPacket -> {
//                            intakeMotor.setVelocity(0);
//                            this.setOuttakeSpeed(lowVelocity);
//                            return false;
//                        },
//                                //Prepare the turret before doing intake, so it can reduce the aiming time
//                                this.getAimAction(-40d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
//                )
                .strafeToConstantHeading(new Vector2d(12, 48))                     //intake
                .strafeToConstantHeading(new Vector2d(-12, 12))//to launch spot
//                .stopAndAdd(new SequentialAction(
//                        this.getAimAction(-30d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, true),
//                        this.getLaunchAction()
//                ))
                .strafeToConstantHeading(new Vector2d(-12, 35))                     //park outside launch
                .build();


    myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}