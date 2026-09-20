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

@TeleOp(name = "AprilTag Rotate Test", group = "Test")
public class AprilTag_test extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        // -------------------------
        // Motors
        // -------------------------

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Typical mecanum configuration.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

                    int tagId = tag.id;
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
        backLeft.setPower(power);

        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    private void stopRobot() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
