package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action redClosePathAction = myBot.getDrive().actionBuilder(new Pose2d(-58.3, -44.5, Math.toRadians(55)))
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(47))   //to launch spot
//                .afterTime(0, this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(-12, -24), Math.toRadians(-90))   //change heading
//                .afterDisp(1, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(-12, -48))                     //to intake
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(47))  //to launch spot
//                .afterTime(0, this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
                .build();

        myBot.runAction(redClosePathAction);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}