

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoBlueSideClose1Stack extends AbstractFullAuto {
    protected ElapsedTime turretTimer = new ElapsedTime();

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, -45, Math.toRadians(55));
    }



    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(-90))//to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, -24))   //change heading
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(-12, -48))                     //to intake
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
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
            return false;
        };
    }

    private void setOuttakePower() {
//        outtakemotorright.setPower(-0.4);
//        outtakemotorleft.setPower(0.4);
        outtakemotor1.setVelocity(-965);
        outtakemotor2.setVelocity(965);

    }

}