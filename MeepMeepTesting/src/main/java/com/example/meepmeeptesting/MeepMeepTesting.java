package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

        Action action = myBot.getDrive().actionBuilder(new Pose2d(63, 15, Math.toRadians(0)))
                .setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(53, 15), Math.toRadians(-22)) //to launch spot
//                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(36, 24), Math.toRadians(90))
//                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(36, 54), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToSplineHeading(new Vector2d(53, 15), Math.toRadians(-22)) //to launch spot
//                .stopAndAdd(this.getLaunchAction())

                .strafeToSplineHeading(new Vector2d(12, 24), Math.toRadians(90))
//                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(12, 54), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToSplineHeading(new Vector2d(53, 15), Math.toRadians(-22)) //to launch spot
//                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(36, 30))//park outside launch
                .build();


        myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}