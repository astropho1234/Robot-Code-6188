package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Rotate Test", group = "Test") //this is strait AI plus like 3 april tag samples so like might run it needs to be tested
public class AprilTag_test extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    @Override
    public void runOpMode() {

        // -------------------------
        // Motors
        // -------------------------

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        rearLeft = hardwareMap.get(DcMotor.class, "rearleft");
        rearRight = hardwareMap.get(DcMotor.class, "rearright");

        // Typical mecanum configuration.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        // -------------------------
        // AprilTag
        // -------------------------

        AprilTagProcessor aprilTagProcessor =
                AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor
        );

        telemetry.addLine("AprilTag initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTagProcessor.getDetections();

            boolean foundTag = false;

            for (AprilTagDetection detection : detections) {

                // SDK 12+
                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection tag =
                            (AprilTagSingleDetection) detection;

                    foundTag = true;

                    int tagId = tag.id; // might need to filter by id because of multiple april tags on the hive
                    double bearing = tag.ftcPose.bearing;

                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("Bearing", "%.1f°", bearing);
                    telemetry.addData(
                            "Range",
                            "%.1f in",
                            tag.ftcPose.range
                    );

                    // -------------------------
                    // Rotate toward tag
                    // -------------------------

                    double tolerance = 2.0;

                    if (Math.abs(bearing) <= tolerance) {

                        stopRobot();

                        telemetry.addLine("TAG CENTERED");

                    } else {

                        // Proportional control
                        double turnPower = bearing * 0.015;

                        // Limit speed
                        turnPower = Math.max(
                                -0.5,
                                Math.min(0.5, turnPower)
                        );

                        // Prevent extremely small power
                        // from failing to move the robot.
                        if (Math.abs(turnPower) < 0.15) {
                            turnPower = Math.copySign(
                                    0.15,
                                    turnPower
                            );
                        }

                        rotate(turnPower);

                        telemetry.addData(
                                "Turn Power",
                                "%.2f",
                                turnPower
                        );
                    }

                    break;
                }
            }

            if (!foundTag) {
                stopRobot();
                telemetry.addLine("No AprilTag detected");
            }

            telemetry.update();
        }

        stopRobot();
        visionPortal.close();
    }

    private void rotate(double power) {

        // Rotate in place.
        frontLeft.setPower(power);
        rearLeft.setPower(power);

        frontRight.setPower(-power);
        rearRight.setPower(-power);
    }

    private void stopRobot() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
}
