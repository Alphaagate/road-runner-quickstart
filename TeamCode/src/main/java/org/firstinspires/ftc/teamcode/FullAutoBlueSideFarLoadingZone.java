package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoBlueSideFarLoadingZone extends AbstractFullAuto {
    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, -15, Math.toRadians(0));
    }
    @Override
    protected Action getPathAction() {
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(53, -15), Math.toRadians(22)) //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(63, -24), Math.toRadians(-90))
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(63, -64), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToSplineHeading(new Vector2d(53, -15), Math.toRadians(22)) //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(36, -30))//park outside launch
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
//        outtakemotorright.setPower(-0.44);
//        outtakemotorleft.setVelocity(0.44);
        outtakeMotor1.setVelocity(-highVelocity);
        outtakeMotor2.setVelocity(highVelocity);
    }
}