package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;

/**
 * This is the DiffySwerve class. It is an implementation of the `SwervePod` interface. It owns the
 * drive motors, analog encoder and the pod rotation PIDF controller.
 *
 * @author Evan Beck - 9925 Delta Robotics
 */

public class DiffyPod implements SwervePod {
    private final AnalogInput turnEncoder;
    private final boolean encoderReversed;
    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;

    private final PIDFController turnPID;
    private final Pose offset;

    private final double angleOffsetRad;
    private final String topMotorLabel;
    private final String bottomMotorLabel;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;

    private double motorCachingThreshold = 0.01;

    private double lastTopPower = 0;
    private double lastBottomPower = 0;

    /**
     *
     * @param encoderName analog encoder name
     * @param topMotorName top motor name
     * @param bottomMotorName bottom motor name
     * @param topMotorDirection top motor direction
     * @param bottomMotorDirection bottom motor direction
     * @param turnPIDFCoefficients PIDF coefficient for turning
     * @param angleOffsetRad raw encoder angle offset in radians. Raw angle in radians when the
     *                       wheel is facing forwards.
     * @param podOffset pod offset from robot center, using the same axes as odometry pods
     * @param analogMinVoltage minimum encoder voltage
     * @param analogMaxVoltage maximum encoder voltage
     * @param encoderReversed true if encoder voltage CCW viewed top-down
     */

    public DiffyPod(
            HardwareMap hardwareMap, String encoderName, String topMotorName,
            String bottomMotorName, DcMotorSimple.Direction topMotorDirection,
            DcMotorSimple.Direction bottomMotorDirection, PIDFCoefficients turnPIDFCoefficients,
            double angleOffsetRad, Pose podOffset, double analogMinVoltage, double analogMaxVoltage,
            boolean encoderReversed) {
        this.turnEncoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.topMotor = hardwareMap.get(DcMotorEx.class, topMotorName);
        this.bottomMotor = hardwareMap.get(DcMotorEx.class, bottomMotorName);
        this.turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;
        this.offset = podOffset;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.encoderReversed = encoderReversed;

        this.topMotorLabel = topMotorName + " (" + topMotor.getConnectionInfo() + ")";
        this.bottomMotorLabel = bottomMotorName + " (" + bottomMotor.getConnectionInfo() + ")";

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topMotor.setDirection(topMotorDirection);
        bottomMotor.setDirection(bottomMotorDirection);
    }


    /**
     * Returns the pod's offset from the robot center.
     *
     * @return offset as Pose
     */
    @Override
    public Pose getOffset() {
        return offset;
    }

    /**
     * Returns the pod's heading after applying the offset, in radians.
     *
     * @return pod heading in radians
     */
    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    /**
     * Converts wheel-space theta (radians) to encoder-space theta.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = encoderReversed ? wheelTheta : (2 * Math.PI - wheelTheta);
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return MathFunctions.normalizeAngle(t);
    }

    /**
     * Move pod to a wheel heading in radians with a drive power [-1,1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges true to suppress turn power
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        // Normalize hardware angle in radians
        double actualRad = MathFunctions.normalizeAngle(getAngleAfterOffsetRad());

        // Adjust polarity dependent on encoder direction
        double desiredRad = adjustThetaForEncoder(targetAngleRad);

        // Get signed error
        double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
        double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // Flip direction and turn to desiredRad + pi if shortest path is greater than pi / 2
        if (Math.abs(errorRad) > (Math.PI  / 2)) {
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;

            mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
            dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
            errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;
        }

        // PID loop
        if (Math.abs(errorRad) < (Math.PI * 2) / 180) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(errorRad);
        double turnPower = ignoreAngleChanges ? 0 : MathFunctions.clamp(turnPID.run(), -1, 1);

        setMotorPowers(turnPower + drivePower, turnPower - drivePower);
    }

    public void setTopPower(double power) {
        lastTopPower = power;
        topMotor.setPower(power);
    }

    public void setBottomPower(double power) {
        lastBottomPower = power;
        bottomMotor.setPower(power);
    }

    private void setMotorPowers(double topPower, double bottomPower) {
        double maxMagnitude = Math.max(1.0, Math.max(Math.abs(topPower), Math.abs(bottomPower)));
        topPower /= maxMagnitude;
        bottomPower /= maxMagnitude;

        if (Math.abs(topPower - lastTopPower) > motorCachingThreshold || (topPower == 0 && lastTopPower != 0)) {
            setTopPower(topPower);
        }

        if (Math.abs(bottomPower - lastBottomPower) > motorCachingThreshold || (bottomPower == 0 && lastBottomPower != 0)) {
            setBottomPower(bottomPower);
        }
    }

    /**
     * Sets drive motors' zero power behavior to FLOAT
     */
    @Override
    public void setToFloat() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets drive motors' zero power behavior to BRAKE
     */
    @Override
    public void setToBreak() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    public double getRawAngleRad() {
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0) {
            return 0;
        }

        double normalized = (turnEncoder.getVoltage() - analogMinVoltage) / range;
        normalized = MathFunctions.clamp(normalized, 0, 1);
        return normalized * (2.0 * Math.PI);
    }

    @Override
    public String debugString() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();
        return topMotorLabel + " {power=" + topMotor.getPower() + "}, "
                + bottomMotorLabel + " {power=" + bottomMotor.getPower() + "}"
                + "\nraw angle (rad/deg)=" + rawAngleRad + " / " + Math.toDegrees(rawAngleRad)
                + "\nangle after offset (rad/deg)=" + offsetAngleRad + " / " + Math.toDegrees(offsetAngleRad);
    }
}