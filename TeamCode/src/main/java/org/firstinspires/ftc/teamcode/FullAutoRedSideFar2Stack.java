package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoRedSideFar2Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_RED;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, 15, Math.toRadians(180));
    }
    @Override
    protected Action getPathAction() {
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(53, 15)) //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToSplineHeading(new Vector2d(36, 24), Math.toRadians(90))
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(36, 54), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToConstantHeading(new Vector2d(53, 15)) //to launch spot
                .stopAndAdd(this.getLaunchAction())

                .strafeToSplineHeading(new Vector2d(12, 24), Math.toRadians(90))
                .afterDisp(0, this.getIntakeAction())
                .strafeToConstantHeading(new Vector2d(12, 54), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToConstantHeading(new Vector2d(53, 15)) //to launch spot
                .stopAndAdd(this.getLaunchAction())
                .strafeToConstantHeading(new Vector2d(36, 30))//park outside launch
                .build();

    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_FAR, 0, 0, NEW_F_FAR);
    }
    @Override
    protected Action getAimAction(double degree) {
        return telemetryPacket -> {
            this.turretMotor.setTargetPosition(convertToTicks(25));
            return false;
        };
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