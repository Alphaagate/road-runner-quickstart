

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "Autonomous")
public class AutoAimTest extends AbstractFullAuto {
    protected ElapsedTime turretTimer = new ElapsedTime();
    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    protected static final int DESIRED_TAG_ID = 24;       // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int TURRET_GEAR_COUNT = 200;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final double MAX_TURRET_TURN_POWER = 0.3;
    private DcMotorEx turretMotor;

    private double lastTargetPositionToMove = 0.0;
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
                .stopAndAdd(this.getAimAction(0))
                .build();
    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

    @Override
    protected Action getAimAction(double degree) {
        return telemetryPacket -> {

            if (useAprilTag) {
                for (int i = 0; i< 50 ; i++ ) {
//                this.moveTurret(convertToTicks(degree));
                    telemetry.addData("Count: ", i);
                    this.detectAprilTag();
                    this.aimAtTarget();
                    telemetry.update();
                    sleep(500);
                }

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
        outtakeMotor1.setVelocity(-lowVelocity);
        outtakeMotor2.setVelocity(lowVelocity);

    }
}