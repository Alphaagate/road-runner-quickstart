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
@Autonomous(name = "FULL_AUTO_BLUE_FAR_PIXEL", group = "Autonomous")
public class FullAutoBlueSideFar extends AbstractFullAuto {


    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, -15, Math.toRadians(0));
    }

    @Override
    public void setOuttakePower() {
        outtakemotorright.setPower(-0.44);
        outtakemotorleft.setVelocity(0.44);
//        outtakemotorright.setVelocity(-1120);
//        outtakemotorleft.setVelocity(1120);
    }

    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(53, -15))
//                .turn(Math.toRadians(20));    //option 1

//                .splineToSplineHeading(new Pose2d(53, -15, Math.toRadians(20)), Math.toRadians(180));   //option 2

                .strafeToSplineHeading(new Vector2d(53, -15), Math.toRadians(22)); //option 3


        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);
    }

    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;

        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
//                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(-90))
//                .turn(Math.toRadians(-110));      //Option 1

//                .splineToSplineHeading(new Pose2d(36, -15, Math.toRadians(-90)), Math.toRadians(-90));   //Option 2
                .strafeToSplineHeading(new Vector2d(36, -15), Math.toRadians(-90)); //option 3

        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(1);
        transfermotor.setPower(-0.5);

        //   old two steps path intake and launch
//        TrajectoryActionBuilder adjustIntakePos = drive.actionBuilder(getCurrentPos(drive))
//                .strafeToConstantHeading(new Vector2d(36, -50), new TranslationalVelConstraint(30.0))
//                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(0));
//        trajectoryActionChosen = adjustIntakePos.build();
//        Actions.runBlocking(trajectoryActionChosen);

//        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
//
//                .strafeToConstantHeading(new Vector2d(53, -15))
//                .turn(Math.toRadians(110));
//        trajectoryActionChosen = goToLaunchSpot2.build();
//        Actions.runBlocking(trajectoryActionChosen);


        // New step path to intake and launch spot
        TrajectoryActionBuilder adjustIntakePos = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(36, -64), new TranslationalVelConstraint(30.0))  // to intake spot
                .strafeToSplineHeading(new Vector2d(53, -15), Math.toRadians(22));   //to launch spot

        trajectoryActionChosen = adjustIntakePos.build();
        Actions.runBlocking(trajectoryActionChosen);
        intakemotor.setPower(0);
        transfermotor.setPower(0);




    }
    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(36, -30));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }

}