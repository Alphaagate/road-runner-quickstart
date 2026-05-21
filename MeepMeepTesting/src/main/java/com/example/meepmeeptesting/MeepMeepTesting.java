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

        Action action = myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeToConstantHeading(new Vector2d(24, 0))//to launch spot
                .turn(Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(24, 24))  //turn before intake

                .turn(Math.toRadians(90))



                .strafeToConstantHeading(new Vector2d(-24, 24))  //turn before intake
                .turn(Math.toRadians(90))

                .strafeToConstantHeading(new Vector2d(-24, -24))  //to launch spot 1st time
                .turn(Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(24, -24))  //to launch spot 1st time

                .build();
    myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}