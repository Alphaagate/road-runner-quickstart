

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "Autonomous")
public class FullAutoBlueSideClose1Stack extends AbstractFullAuto {
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
        outtakeMotor1.setVelocity(-lowVelocity);
        outtakeMotor2.setVelocity(lowVelocity);

    }
}