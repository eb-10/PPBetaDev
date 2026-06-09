package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Differential swerve pod implementation for Pedro Pathing 2.1.2.
 *
 * <p>Angles passed to and returned from SwervePod are radians. The motor mixing assumes opposite
 * motor powers drive the wheel and matching motor powers rotate the module:
 * top = drive + turn, bottom = -drive + turn.
 */
public class DifferentialPod implements SwervePod {
    private final AnalogInput turnEncoder;
    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;

    private final PIDFController turnPID;
    private final Pose offset;

    private final double angleOffsetRad;
    private final String topMotorLabel;
    private final String bottomMotorLabel;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final boolean encoderReversed;

    private double motorCachingThreshold = 0.01;
    private double lastTopPower = 0;
    private double lastBottomPower = 0;

    /**
     * @param topMotorName top differential motor hardware-map name
     * @param bottomMotorName bottom differential motor hardware-map name
     * @param turnEncoderName analog encoder hardware-map name
     * @param turnPIDFCoefficients PIDF coefficients for steering control, tuned for radian error
     * @param topDirection top motor direction
     * @param bottomDirection bottom motor direction
     * @param angleOffsetRad raw encoder angle, in radians, when the wheel is facing forward
     * @param podOffset pod position offset from robot center, using Pedro's odometry coordinate axes
     * @param analogMinVoltage minimum observed encoder voltage
     * @param analogMaxVoltage maximum observed encoder voltage
     * @param encoderReversed true if encoder increases CCW when viewed from above
     */
    public DifferentialPod(
            HardwareMap hardwareMap,
            String topMotorName,
            String bottomMotorName,
            String turnEncoderName,
            PIDFCoefficients turnPIDFCoefficients,
            DcMotorSimple.Direction topDirection,
            DcMotorSimple.Direction bottomDirection,
            double angleOffsetRad,
            Pose podOffset,
            double analogMinVoltage,
            double analogMaxVoltage,
            boolean encoderReversed) {

        this.topMotor = hardwareMap.get(DcMotorEx.class, topMotorName);
        this.bottomMotor = hardwareMap.get(DcMotorEx.class, bottomMotorName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        this.topMotorLabel = topMotorName + " (" + topMotor.getConnectionInfo() + ")";
        this.bottomMotorLabel = bottomMotorName + " (" + bottomMotor.getConnectionInfo() + ")";

        this.turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;
        this.offset = podOffset;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.encoderReversed = encoderReversed;

        setMotorToFloat();
        topMotor.setDirection(topDirection);
        bottomMotor.setDirection(bottomDirection);
        setMotorPowers(0, 0);
    }

    @Override
    public Pose getOffset() {
        return offset;
    }

    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    public void setTopPower(double power) {
        lastTopPower = power;
        topMotor.setPower(power);
    }

    public void setBottomPower(double power) {
        lastBottomPower = power;
        bottomMotor.setPower(power);
    }

    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    public void setMotorToFloat() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBreak() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isEncoderReversed() {
        return encoderReversed;
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        double adjustedTheta = encoderReversed ? wheelTheta : (2.0 * Math.PI - wheelTheta);
        return MathFunctions.normalizeAngle(adjustedTheta);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        double actualRad = MathFunctions.normalizeAngle(getAngleAfterOffsetRad());
        double desiredRad = adjustThetaForEncoder(targetAngleRad);

        double errorRad = getSignedAngleError(actualRad, desiredRad);

        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;
            errorRad = getSignedAngleError(actualRad, desiredRad);
        }

        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(errorRad);
        double turnPower = ignoreAngleChanges ? 0 : MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        setMotorPowers(drivePower + turnPower, -drivePower + turnPower);
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

    public double getOffsetAngleRad() {
        return MathFunctions.normalizeAngle(getAngleAfterOffsetRad());
    }

    public void setMotorCachingThreshold(double threshold) {
        this.motorCachingThreshold = threshold;
    }

    private double getSignedAngleError(double actualRad, double desiredRad) {
        double magnitude = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double direction = MathFunctions.getTurnDirection(actualRad, desiredRad);
        return magnitude == Math.PI ? -Math.PI : magnitude * direction;
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