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
public class FullAutoBlueSideFar1Stack extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, -15, Math.toRadians(180));
    }
    @Override
    protected Action getPathAction() {
        return drive.actionBuilder(getInitialPose())
                .setTangent(Math.toRadians(180))
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                    this.setOuttakeSpeed();

                    return false;
                },this.getAimAction(20)))
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(45),
                        this.getLaunchAction()
                ))
                .afterDisp(0, telemetryPacket -> {
                    this.reverseOuttake();
                    return false;
                })                .strafeToSplineHeading(new Vector2d(36, -24), Math.toRadians(-90))
                .afterDisp(0, new SequentialAction(
                        telemetryPacket -> {
                            this.reverseOuttake();
                            return false;
                        }, this.getIntakeAction()

                ))
                .strafeToConstantHeading(new Vector2d(36, -54), new TranslationalVelConstraint(30.0))  // to intake spot
                .afterDisp(0, telemetryPacket -> {
                    intakeMotor.setVelocity(0);
                    this.setOuttakeSpeed();
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(53, -15)) //to launch spot
                .stopAndAdd(new SequentialAction(
                        this.getAimAction(-90),
                        this.getLaunchAction()
                ))
                .strafeToConstantHeading(new Vector2d(36, -30))//park outside launch
                .build();
    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_FAR, 0, 0, NEW_F_FAR);
    }

    @Override
    protected Action getAimAction(double degree) {
        return telemetryPacket -> {

            if (useAprilTag) {
                if (degree != 0){
                    this.moveTurret(convertToTicks(degree));
                }
                this.detectAprilTag();
                this.aimAtTarget();
            } else {
                this.moveTurret(convertToTicks(degree));
                //TODO: change pos
                this.moveHoodServo(0.1);
            }
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

//            this.sleep(700);
            //blockservo not using yet yet
//            blockServo.setPosition(1);
            this.sleep(500);
            this.intakeMotor.setPower(1);
            this.sleep(1000);
            this.intakeMotor.setPower(0);
            //wait for autoaim
//            this.sleep(2000);

            return false;
        };
    }

    private void setOuttakeSpeed() {
        this.sleep(100);
        outtakeMotor1.setVelocity(-highVelocity);
        outtakeMotor2.setVelocity(highVelocity);

    }
}