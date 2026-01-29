

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
public class FullAutoBlueSideClose2StackNoGate extends AbstractFullAuto {
    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID_BLUE;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-58.3, -45, Math.toRadians(235));
    }
    @Override
    protected Action getPathAction() {

        return drive.actionBuilder(getInitialPose())
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            return false;
                        },
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(null, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 1st time
                .stopAndAdd(new SequentialAction(
                        // adjust by using AprilTag again
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        telemetryPacket -> {
                            this.sleep(300);
                            this.blockDown();
                            return false;
                        }
                ))

                .strafeToSplineHeading(new Vector2d(-12, -24), Math.toRadians(-85))   //change heading

                .afterDisp(0, new SequentialAction(telemetryPacket -> {
                            this.intakeMotor.setPower(1);
                            this.setOuttakeSpeed(0);
                            return false;
                        })
                )
                .strafeToConstantHeading(new Vector2d(-12, -54), new TranslationalVelConstraint(20))  //to intake 1st stack
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            this.sleep(200);
                            this.blockUp();
                            return false;
                        },
                        //Prepare the turret before doing intake, so it can reduce the aiming time
                        this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 2nd

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction(),
                        telemetryPacket -> {
                            this.sleep(300);
                            this.blockDown();
                            return false;
                        }
                ))
                .strafeToSplineHeading(new Vector2d(12, -24), Math.toRadians(-85), new TranslationalVelConstraint(20))  //turn before intake

                .afterDisp(0, new SequentialAction(telemetryPacket -> {
                            this.intakeMotor.setPower(1);
                            this.setOuttakeSpeed(0);
                            return false;
                        })
                )
                .strafeToConstantHeading(new Vector2d(12, -54), new TranslationalVelConstraint(15))      //intake 2nd stack
                .afterDisp(0, new ParallelAction(telemetryPacket -> {
                            intakeMotor.setVelocity(0);
                            this.setOuttakeSpeed(lowVelocity);
                            this.sleep(200);
                            this.blockUp();
                            return false;
                        },
                                //Prepare the turret before doing intake, so it can reduce the aiming time
                                this.getAimAction(-35d, HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE, false))
                )
                .strafeToConstantHeading(new Vector2d(-12, -12))  //to launch spot 3rd time

                .stopAndAdd(new SequentialAction(
                        this.getAimAction(null, null, true),
                        this.getLaunchAction()
                ))
                .strafeToConstantHeading(new Vector2d(-12, -35))                     //park outside launch
                .build();
    }

    @Override
    protected double getTurretDegreeOffset() {
        return 0;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }


}