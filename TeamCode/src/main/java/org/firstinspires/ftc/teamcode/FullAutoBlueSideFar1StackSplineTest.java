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
public class FullAutoBlueSideFar1StackSplineTest extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, -15, Math.toRadians(180));
    }
    @Override
    protected Action getPathAction() {

        double xOffset = 5;
        double yOffset = 8;
        double degreeOffset = 10;
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))

                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot

                .strafeToSplineHeading(new Vector2d(36 + xOffset, -24), Math.toRadians(-90 + degreeOffset))

                .strafeToConstantHeading(new Vector2d(36 + xOffset, -58 + yOffset), new TranslationalVelConstraint(30.0))  // to intake spot

                .strafeToConstantHeading(new Vector2d(53, -15 + yOffset)) //to launch spot

                .build();

    }

    @Override
    protected double getTurretDegreeOffset() {
        return 0;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_FAR, 0, 0, NEW_F_FAR);
    }

}