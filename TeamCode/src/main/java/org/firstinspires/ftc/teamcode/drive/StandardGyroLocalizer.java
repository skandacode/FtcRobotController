package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class StandardGyroLocalizer extends GyroTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.68897637795275590551181102362205; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double X_MULTIPLIER = 1, Y_MULTIPLIER = 1;

    public static double LATERAL_DISTANCE = 10.585861; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7.076; // in; offset of the lateral wheel
    public static boolean ENABLE_IMU = true;
    public static double GYRO_TRUST = 1;

    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardGyroLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ), ENABLE_IMU ? hardwareMap.get(IMU.class, "imu") : null);
        setGyroConfidence(GYRO_TRUST);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontleft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backright"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontright"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        double leftVel = leftEncoder.getCorrectedVelocity();
        double rightVel = rightEncoder.getCorrectedVelocity();
        double frontVel = frontEncoder.getCorrectedVelocity();

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}