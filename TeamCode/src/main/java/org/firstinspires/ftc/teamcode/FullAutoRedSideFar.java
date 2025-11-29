package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "FULL_AUTO_RED_FAR_PIXEL", group = "Autonomous")
public class FullAutoRedSideFar extends AbstractFullAuto {


    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, 15, Math.toRadians(0));

    }

    @Override
    public void setOuttakePower() {
        outtakemotorright.setPower(-0.35);
        outtakemotorleft.setPower(0.35);
    }

    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(53, 15))
//                .turn(Math.toRadians(-22));
                .strafeToSplineHeading(new Vector2d(53, 15), Math.toRadians(-20)); //option 3

        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);


    }

    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
//                .strafeToConstantHeading(new Vector2d(36, 20))
//                .turn(Math.toRadians(112));
                .splineToSplineHeading(new Pose2d(36, 15, Math.toRadians(90)), Math.toRadians(90));   //Option 2

        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(1);
        transfermotor.setPower(-1);

        TrajectoryActionBuilder adjustIntakePos = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(36, 50), new TranslationalVelConstraint(30.0))
//                .splineToConstantHeading(new Vector2d(36, 15), Math.toRadians(0));
                .strafeToSplineHeading(new Vector2d(53, 15), Math.toRadians(-20));   //to launch spot

        trajectoryActionChosen = adjustIntakePos.build();
        Actions.runBlocking(trajectoryActionChosen);
//        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
//
//
//        trajectoryActionChosen = goToLaunchSpot2.build();
//        Actions.runBlocking(trajectoryActionChosen);
    }

    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(36, 30));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }

}