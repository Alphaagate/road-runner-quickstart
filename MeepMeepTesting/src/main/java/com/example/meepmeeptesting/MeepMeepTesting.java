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
                .setConstraints(72.5, 72.5, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action action = myBot.getDrive().actionBuilder(new Pose2d(-58.3, -45, Math.toRadians(235)))

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time

                .strafeToSplineHeading(new Vector2d(12, -18), Math.toRadians(-90))  //turn before intake


                .strafeToConstantHeading(new Vector2d(12, -54))      //intake 2nd stack

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time

                .strafeToConstantHeading(new Vector2d(12, -36))

                .strafeToSplineHeading(new Vector2d(10, -56), Math.toRadians(-115))
                .splineTo(new Vector2d(10, -56), Math.PI / 2)


                .strafeToConstantHeading(new Vector2d(12, -36))

                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(-90))  //to launch spot 2nd

               // .strafeToSplineHeading(new Vector2d(-12, -24), Math.toRadians(-90))   //change heading


                .strafeToConstantHeading(new Vector2d(-12, -54))  //to intake 1st stack

                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 2nd



                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
                .build();

    myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}