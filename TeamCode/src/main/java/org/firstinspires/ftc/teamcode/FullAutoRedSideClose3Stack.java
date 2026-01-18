

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoRedSideClose3Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, 45, Math.toRadians(-235));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .strafeToConstantHeading(new Vector2d(-12, 12))//to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(90))   //change heading
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(-12, 48))                     //to intake
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(12, 24), Math.toRadians(90))  //to launch spot
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(12, 48))                     //intake
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(36, 24), Math.toRadians(90))  //to launch spot
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(36, 48))                     //intake
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, 35))                     //park outside launch
                .build();
    }

    
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

}