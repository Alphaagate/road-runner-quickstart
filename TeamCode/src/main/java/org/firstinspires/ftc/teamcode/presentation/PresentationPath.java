

package org.firstinspires.ftc.teamcode.presentation;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.AbstractFullAuto;

@Config
@Autonomous(group = "Autonomous")
public class PresentationPath extends AbstractFullAuto {

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, -45, Math.toRadians(235));
    }

    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
//                .afterDisp(0, this.setOuttakeSpeed(1250))
                .afterDisp(0, new SequentialAction(telemetryPacket -> {
                            this.setOuttakeSpeed(lowVelocity);
                            return false;
                        }, this.getIntakeAction())
                )
                .strafeToSplineHeading(new Vector2d(-72, 24), Math.toRadians(90))  //to launch spot
                .stopAndAdd(new SequentialAction(
                        //Further adjust the aiming before launching

                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                .build();
    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }


}