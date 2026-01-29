

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(group = "Autonomous")
public class AutoAimTest extends AbstractFullAuto {

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
                .stopAndAdd(this.getAimAction(0d))
                .build();
    }
    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return new PIDFCoefficients(NEW_P_CLOSE, 0, 0, NEW_F_CLOSE);
    }

    protected Action getAimAction(Double degree) {
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
    protected double getTurretDegreeOffset() {
        return 0d;
    }

}