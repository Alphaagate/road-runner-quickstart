

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "FULL_AUTO_RED_ClOSE_PIXEL", group = "Autonomous")
public class FullAutoRedSideClose2Stack extends AbstractFullAuto {
    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, 44.5, Math.toRadians(-55));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(-47))//to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(90))   //change heading
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(-12, 48))                     //to intake
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(-47))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(12, 24), Math.toRadians(90))  //to launch spot
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(12, 48))                     //intake
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(-47))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, 35))                     //park outside launch
                .build();
    }

    @Override
    protected Action getLaunchAction() {

//        Action launchAction = new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                this.setOuttakePowerForClose();
//                this.kickBalls();
//                return false;
//            }
//        };
//        return launchAction;

        return telemetryPacket -> {
            this.setOuttakePower();
            this.kickBalls();
            return false;
        };
    }
    private void setOuttakePower() {
        outtakemotorright.setVelocity(-965);
        outtakemotorleft.setVelocity(965);
    }
}