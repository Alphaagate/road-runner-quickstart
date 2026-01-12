

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoRedSideClose1Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, 44.5, Math.toRadians(-235));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
//                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(90))   //change heading
//                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(-12, 48))                     //to intake
                .strafeToConstantHeading(new Vector2d(-12, 12))  //to launch spot
//                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(-12, 35))                     //park outside launch
                .build();
    }

    @Override
    protected Action getAimAction() {
        return telemetryPacket -> {
            this.turretMotor.setTargetPosition(convertToTicks(55));
            return false;
        };
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
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
        outtakeMotor1.setVelocity(-lowVelocity);
        outtakeMotor2.setVelocity(lowVelocity);
    }
}