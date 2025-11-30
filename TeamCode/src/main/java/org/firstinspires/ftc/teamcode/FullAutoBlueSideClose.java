

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "FULL_AUTO_BLUE_ClOSE_PIXEL", group = "Autonomous")
public class FullAutoBlueSideClose extends AbstractFullAuto {


    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-57, -47, Math.toRadians(55));

    }

    @Override
    public void setOuttakePower() {
//        outtakemotorright.setPower(-0.4);
//        outtakemotorleft.setPower(0.4);
        outtakemotorright.setVelocity(-965);
        outtakemotorleft.setVelocity(965);

    }

    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(35))
                .strafeToConstantHeading(new Vector2d(-12, -12));
//                .turn(Math.toRadians(-7));

//                .splineToConstantHeading(new Vector2d(-9, -23), Math.toRadians(-90));
        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);
    }


    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(-159.1))
                .strafeToConstantHeading(new Vector2d(-24, -40));
        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);
        intakemotor.setPower(1);
        transfermotor.setPower(-0.75);
        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
//                .turn(Math.toRadians(159.1))
//                .strafeToConstantHeading(new Vector2d(-12, -23));
                .strafeToSplineHeading(new Vector2d(-24, 0), Math.toRadians(55));   //to launch spot


        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);


    }
    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(0));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }


}