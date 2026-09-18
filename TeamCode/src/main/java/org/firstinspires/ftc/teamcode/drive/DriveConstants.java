package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    /*
     * Webots / drive motor constants
     */
    public static final double TICKS_PER_REV = 560;
    public static final double MAX_RPM = 300;

    /*
     * Disable built-in motor velocity PID for Webots.
     */
    public static final boolean RUN_USING_ENCODER = false;

    public static PIDFCoefficients MOTOR_VELO_PID =
            new PIDFCoefficients(
                    0,
                    0,
                    0,
                    getMotorVelocityF(
                            MAX_RPM / 60.0 * TICKS_PER_REV
                    )
            );

    /*
     * Physical constants
     */
    public static double WHEEL_RADIUS = 1.48; // inches

    /*
     * Start with 1.0 for the Webots direct-drive setup.
     */
    public static double GEAR_RATIO = 1.0;

    /*
     * Approximate starting value.
     * Tune this later.
     */
    public static double TRACK_WIDTH = 14.0; // inches

    /*
     * Feedforward
     */
    public static double kV =
            1.0 / rpmToVelocity(MAX_RPM);

    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * Trajectory constraints.
     * Keep these conservative while testing.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;

    public static double MAX_ANG_VEL =
            Math.toRadians(60);

    public static double MAX_ANG_ACCEL =
            Math.toRadians(60);

    /*
     * IMU orientation
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection
            LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    public static RevHubOrientationOnRobot.UsbFacingDirection
            USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static double encoderTicksToInches(double ticks) {

        return WHEEL_RADIUS
                * 2.0
                * Math.PI
                * GEAR_RATIO
                * ticks
                / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {

        return rpm
                * GEAR_RATIO
                * 2.0
                * Math.PI
                * WHEEL_RADIUS
                / 60.0;
    }

    public static double getMotorVelocityF(
            double ticksPerSecond) {

        if (ticksPerSecond <= 0
                || !Double.isFinite(ticksPerSecond)) {

            return 0;
        }

        return 32767.0 / ticksPerSecond;
    }


}