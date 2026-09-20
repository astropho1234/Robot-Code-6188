package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "MeepMeep Strafe Test")
public class MeepMeepTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive =
                new SampleMecanumDrive(hardwareMap);

        Pose2d startPose =
                new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory =
                drive.trajectorySequenceBuilder(startPose)
                        .strafeLeft(50)
                        .forward(120)
                        .build();

        telemetry.addLine("Ready - Strafe Left 24 inches");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        drive.followTrajectorySequence(trajectory);
    }
}
