package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is the abstract base class for the HRVHS robots built on the Andymark TileRunner HD Mecanum base. It is <i>mostly</i>
 * a translation of the common parts of the blocks <tt>TestMotors</tt>, <tt>DriveExample</tt>, and <tt>AutoCalibrate</tt>
 * OpModes into a base class that supports extension to each of those OpModes, and to the OpModes you need to build for
 * drive and autonomous programs for FTC SkyStone.
 *
 * NOTE: all function and variable names have been re-formatted to conform to standard Java conventions.
 */
public abstract class AMecBase extends LinearOpMode {

    protected DcMotor     FL;     // The front left motor
    protected DcMotor     FR;     // The front right motor
    protected DcMotor     RR;     // The right rear motor
    protected DcMotor     LR;     // The left rear motor
    protected BNO055IMU   IMU;    // The IMU, generally on the hub controlling the motors

    double ticsPerInchForward;
    double ticsPerInchSideways;
    double mtrAccelMin;
    double mtrAccelTics;
    double mtrAccelDegs;
    double mtrDecelMin;
    double mtrDecelTics;
    double mtrDecelDegs;

    double stickDeadband;
    double stickSensitivity;
    // The gamepad1 stick values conditioned for deadband and sensitivity
    double conditionedRightX;
    double conditionedRightY;
    double conditionedLeftX;
    double conditionedLeftY;

    double kp;

    double heading;
    int headingRevs;
    double headingRawLast;
    double expectedHeading;
    boolean inTurn;


    /**
     * This function initializes all of the control constants your robot. Override this (remembering to call this super) if
     * your want to override any of these constants in a specific implementation or if you have additional constants to initialize
     */
    protected void initConstants() {
        // These will be specific to your robot base, and the gear ratio you selected when you built your robot. Run the
        // calibration and tune these values for your robot.
        ticsPerInchForward = 295;
        ticsPerInchSideways = 350;

        // These values were selected to be reasonable for the TileRunner base with a 1:1 gear ratio. Refer to the README notes
        // and tune these for your robot. Refer to the project readme for notes about the power ramp function being
        // controlled by these variables.
        mtrAccelMin = 0.3;
        mtrAccelTics = 1000;
        mtrAccelDegs = 20;

        mtrDecelMin = 0.2;
        mtrDecelTics = 3000;
        mtrDecelDegs = 30;

        // These are for conditioning stick values from the logitech gamepad to make the robot more controllable
        // for the driver.
        stickDeadband = 0.05;
        stickSensitivity = 2.0;

        // This is the proportional multiplier for the IMU PID loop (we only use P), that tries to maintain the robot heading to
        // the expected heading. NOTE: IMU
        kp = 0.05;
    }

    /**
     * This is generally hardware-specific initialization. it cannot happen until the OpMode has started (<tt>runOpMode()</tt> is
     * called) because you need access to the hardwareMap which is populated from the configuration file before
     * runOpMode() is called
     */
    protected void preStartInitialization() {
        // Initialize the hardware variables from the hardware map
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        RR = hardwareMap.dcMotor.get("RR");
        LR = hardwareMap.dcMotor.get("LR");
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");

        // Initialize the motors for the correct directions and braking
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This initializes the IMU for use of the gyros. NOTE: gyros precess, and the REV hub may not be perfectly
        // aligned with the frame. Make no assumptions about the heading reported from the IMU. Read the IMU heading when
        // the OpMode starts, and use that as the reference.
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        IMU.initialize(imuParams);
        while (opModeIsActive()) {
            if (IMU.isGyroCalibrated()) {
                break;
            }
        }
    }

    /**
     * This is initialization that happens post start, but before the main control loop is run.
     */
    protected void postStartInitialization() {
        // setup heading calculation and save the current heading as the expected heading, i.e. the current heading becomes the
        // reference expected heading fo all actions after this.
        headingRevs = 0;
        headingRawLast = 0.0;
        recomputeHeading();
        expectedHeading = heading;
        inTurn = false;
    }

    /**
     * Reset the encoders for the four drive motors.
     */
    protected void resetDriveEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Set the power for the 4 drive motors.
     *
     * @param powerRF (double) The right front wheel power in the range -1.0 to 1.0.
     * @param powerRR (double) The right rear wheel power in the range -1.0 to 1.0.
     * @param powerLF (double) The left front wheel power in the range -1.0 to 1.0.
     * @param powerLR (double) The left rear wheel power in the range -1.0 to 1.0.
     */
    protected void setPower(final double powerRF, final double powerRR,final double powerLF,final double powerLR) {
        FR.setPower(powerRF);
        RR.setPower(powerRR);
        FL.setPower(powerLF);
        LR.setPower(powerLR);
    }

    /**
     * Set the motor powers to drive the robot with the specified direct and rotation.
     *
     * @param forward (double) The forward power in the range -1.0 to 1.0.
     * @param sideways (double) The sideways power in the range -1.0 to 1.0.
     * @param rotation (double) The rotation power in the range -1.0 to 1.0.
     */
    protected void setDrive(double forward, double sideways, double rotation) {
        double scale = 1.0;
        double max = Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
        if (max > 1.0) {
            scale = 1.0 / max;
        }
        double powerRF = scale * ((forward - sideways) - rotation);
        double powerRR = scale * ((forward + sideways) - rotation);
        double powerLF = scale * (forward + sideways + rotation);
        double powerLR = scale * ((forward - sideways) + rotation);
        setPower(powerRF, powerRR, powerLF, powerLR);
    }

    /**
     * Condition the stick values. What this means is:
     * <ul>
     *     <li>read the stick values;</li>
     *     <li>apply a deadband correction so hear-zero errors and considered to be zero;</li>
     *     <li>apply a sensitivity correction that can 'flatten' the reported stick value around 0 to give better
     *         sensitivity at slow speeds.</li>
     * </ul>
     * NOTE: This default implementation uses the global <tt>stickDeadband</tt> and <tt>stickSensitivity</tt> values
     * to condition both axes of both sticks. Override this method for stick/axis specific corrections.
     */
    protected void conditionSticks() {
        conditionedRightX = conditionStickValue(gamepad1.right_stick_x);
        conditionedRightY = conditionStickValue(-gamepad1.right_stick_y);
        conditionedLeftX = conditionStickValue(gamepad1.left_stick_x);
        conditionedLeftY = conditionStickValue(-gamepad1.left_stick_y);
    }

    /**
     * Condition a stick value using the global <tt>stickDeadband</tt> and <tt>stickSensitivity</tt> values.
     *
     * @param stickValue (double) The raw stick value.
     *
     * @return (double) Returns the conditioned stick value.
     */
    private double conditionStickValue(double stickValue) {
        return conditionStickValue(stickValue, stickDeadband, stickSensitivity);
    }

    /**
     * Condition a raw stick value using the specified <tt>deadband</tt> and <tt>sensitivity</tt>.
     *
     * @param stickValue (double) The raw stick value.
     * @param deadband (double) The distance from 0.0 on the stick that is considered to be 0.0. This compensates for
     *                 small unintended pressures on the stick or errors in the stick returning to the 0 position.
     * @param sensitivity (double, > 0.0) The exponent applied to the stick value after the deadband correction. Values greater
     *                    than 1 flatten the conditioned value curve; that is, it increases the sensitivity around 0. So
     *                    when you move the stick a small amount the actual value is decreased for greater sensitivity
     *                    around 0. Drivers typically like values of 2 to 3 as the sensitivity.
     *
     * @return (double) Returns the conditioned stick value.
     */
    private double conditionStickValue(double stickValue, double deadband, double sensitivity) {
        if (Math.abs(stickValue) < deadband)  {
            return 0.0;
        }
        double stickSign = (stickValue < 0.0) ? -1.0 : 1.0;
        double deadbandCorrected = (Math.abs(stickValue) - deadband) / (1.0 - deadband);
        return stickSign * Math.pow(deadbandCorrected, sensitivity);
    }

    /**
     * Recompute the heading as reported by the IMU and adjusted to be always increasing when rotation is clockwise. This
     * heading computation was introduce by Jason Barringer to the FRC 6831 AO5 Annex code base in the 2017 season to make
     * using PID loops to control heading with the IMU easier to write, and more predictable. If there is a discontinuity
     * in the sensor output, this means there needs to be special logic in the PID code to deal with the discontinuity. This
     * handles the discontinuity in a single place where the heading is computed.
     */
    protected void recomputeHeading() {
        Orientation angles = IMU.getAngularOrientation();
        // This assumes the base of the REV hub is parallel to the ground plane.
        float heading_raw = angles.firstAngle;
        // This is the logic for detecting and correcting for the IMU discontinuity at +180degrees and -180degrees.
        if (headingRawLast < -150.0 && heading_raw > 0.0) {
            // The previous raw IMU heading was negative and close to the discontinuity, and it is now positive. We have
            // gone through the discontinuity so we decrement the heading revolutions by 1 (we completed a negative
            // revolution). NOTE: the initial check protects from the case that the heading is near 0 and goes continuously
            // through 0, which is not the completion of a revolution.
            headingRevs--;
        } else if (headingRawLast > 150.0 && heading_raw < 0.0) {
            // The previous raw IMU heading was positive and close to the discontinuity, and it is now negative. We have
            // gone through the discontinuity so we increment the heading revolutions by 1 (we completed a positive
            // revolution). NOTE: the initial check protects from the case that the heading is near 0 and goes continuously
            // through 0, which is not the completion of a revolution.
            headingRevs++;
        }
        heading = -((headingRevs * 360.0) + heading_raw);
        headingRawLast = heading_raw;
    }

    /**
     * Compute the power for the current location on the ramp-up, hold max, ramp down power curve.
     *
     * @param currentTics (double) The current position relative to the target position. This is typically encoder tics
     *                    for forward or sideways motion, and degrees for rotation.
     * @param targetTics (double) the target (desired final) position. This is typically encoder tics
     *                   for forward or sideways motion, and degrees for rotation.
     * @param maxPower (double) The maximum power that should be applied.
     * @param accelMin (double) The power that should be applied when the initial position is 0.
     * @param accelTics (double) The distance in which the power should increase from <tt>accelMin</tt> to 1.0. This is
     *                  typically encoder tics for forward or sideways motion, and degrees for rotation.
     * @param decelMin (double) The power the motors should decelerate to immediately before the final position is reached.
     * @param decelTics (double) The distance from the final position at which the power should start to decrease from
     *                  1.0 towards <tt>decelMin</tt>. This is typically encoder tics for forward or sideways motion,
     *                  and degrees for rotation.
     * @return (double) Returns the motor power that should be used for the current position given the specified power
     *                  ramp constraints.
     */
    private double powerAccelDecel(double currentTics, double targetTics, double maxPower,
                                   double accelMin, double accelTics, double decelMin, double decelTics) {
        if (currentTics <= 0) {
            return accelMin;
        }
        if (currentTics >= targetTics) {
            return 0.0;
        }
        double mtrPower = maxPower;
        if (currentTics < accelTics) {
            double accelPower = accelMin + ((1 - accelMin) * (currentTics / accelTics));
            if (accelPower < mtrPower) {
                mtrPower = accelPower;
            }
        }
        if (currentTics > targetTics - decelTics) {
            double decelPower = decelMin + ((1 - decelMin) * ((targetTics - currentTics) / decelTics));
            if (decelPower < mtrPower) {
                mtrPower = decelPower;
            }
        }
        return mtrPower;
    }

    /**
     * Get the current accumulated forward (reverse) tics from all 4 drive motors. Heading correction and differences in
     * power to actual speed, and the effects of weight distribution and frame alignment means that all 4 motors
     * will probably have different encoder readings on any encoder controlled
     * move. The number of variable is large enough that picking a single encoder seems error-prone; so we use them all, which
     * should balance out minor differences in performance from run to run.
     *
     * @return (double) The accumulated forward (backward) tics from all 4 drive motors.
     */
    private double forwardTics() {
        // NOTE:, to go forward all 4 drive motors are going forward.
        return FR.getCurrentPosition() + RR.getCurrentPosition() + FL.getCurrentPosition() + LR.getCurrentPosition();
    }

    /**
     * Get the current accumulated sideways tics from all 4 drive motors. Heading correction and differences in
     * power to actual speed, and the effects of weight distribution and frame alignment means that all 4 motors
     * will probably have different encoder readings on any encoder controlled
     * move. The number of variable is large enough that picking a single encoder seems error-prone; so we use them all, which
     * should balance out minor differences in performance from run to run.
     *
     * @return (double) The accumulated sideways (backward) tics from all 4 drive motors.
     */
    private double sidewaysTics() {
        // NOTE:, to go sideways, the front right and left rear drive motors are reversed.
        return (RR.getCurrentPosition() + FL.getCurrentPosition()) - (FR.getCurrentPosition() + LR.getCurrentPosition());
    }

    /**
     * Move in the direction specified for the distance specified. Mecanum drive can move in any direction without
     * rotating (changing heading, or the direction the robot is facing). This move maintains robot heading; i.e. at the
     * end of the move the robot will be facing in the same direction as when it started. When you use this method the
     * robot will accelerate to maximum power ot 1.0.
     *
     * @param inches (double) The distance (in inches) that the robot should move.
     * @param degrees (double) The direction in degrees from straight ahead (which is 0 degrees) in which you want the
     *                robot to move. Clockwise, or to the right being is positive.
     */
    final protected void move(double inches, double degrees) {
        move(inches, degrees, 1.0);
    }

    /**
     * Move in the specified direction for the specified distance without exceeding the specified power. This method is
     * often used when the target is close-at-hand and you want to slow your approach to the target.
     *
     * @param inches (double) The distance (in inches) that the robot should move.
     * @param degrees (double) The direction in degrees from straight ahead (which is 0 degrees) in which you want the
     *                robot to move. Clockwise, or to the right being is positive.
     * @param maxPower {double} The maximum power you want to sent to any of your motors.
     */
    final protected void move(double inches, double degrees, double maxPower) {
        move(inches, degrees, maxPower, mtrAccelMin, mtrAccelTics, mtrDecelMin, mtrDecelTics);
    }

    /**
     * Move in the specified direction for the specified distance without exceeding any of the specified power constraints. This
     * method is most commonly used during competition initializetion to help randomize the environment.
     * @param inches (double) The distance (in inches) that the robot should move.
     * @param degrees (double) The direction in degrees from straight ahead (which is 0 degrees) in which you want the
     *                robot to move. Clockwise, or to the right being is positive.
     * @param maxPower {double} The maximum power you want to sent to any of your motors.
     * @param accelMin (double) The power that should be applied when the initial position is 0.
     * @param accelTics (double) The distance (encoder tics) in which the power should increase from <tt>accelMin</tt> to 1.0.
     * @param decelMin (double) The power the motors should decelerate to immediately before the final position is reached.
     * @param decelTics (double) The distance (encoder tics) from the final position at which the power should start to decrease
     *                  from 1.0 towards <tt>decelMin</tt>.
     */
    final protected void move(double inches, double degrees, double maxPower,
                      double accelMin, double accelTics, double decelMin, double decelTics) {
        resetDriveEncoders();
        double sin = Math.sin(degrees / 180 * Math.PI);
        double cos = Math.cos(degrees / 180 * Math.PI);
        // get the forward and sideways components of the move. To give the greatest accuracy of the move we will use the one
        // that is the longest (greatest number of encoder tics) to decide when the move has completed.
        double forwardMaxSpeed = Math.abs(cos);
        double forwardInches = cos * inches;
        double forwardDirectionMult = (forwardInches > 0.0) ? 1.0 : -1.0;

        double sidewaysMaxSpeed = Math.abs(sin);
        double sidewaysInches = sin * inches;
        double sidewaysDirectionMult = (sidewaysInches > 0) ? 1.0 : -1.0;

        double target_tics = (forwardMaxSpeed >= sidewaysMaxSpeed) ?
                ticsPerInchForward * forwardInches * forwardDirectionMult :
                ticsPerInchSideways * sidewaysInches * sidewaysDirectionMult;
        while (opModeIsActive()) {
            double current_tics = (forwardMaxSpeed >= sidewaysMaxSpeed) ?
                    forwardTics() * forwardDirectionMult : sidewaysTics() * sidewaysDirectionMult;
            if (current_tics >= target_tics) {
                break;
            }
            double speed_mult = powerAccelDecel(current_tics, target_tics, maxPower, accelMin, accelTics, decelMin, decelTics);
            recomputeHeading();
            double error = expectedHeading - heading;
            setDrive(forwardMaxSpeed * speed_mult * forwardDirectionMult,
                    sidewaysMaxSpeed * speed_mult * sidewaysDirectionMult,
                    kp * error);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    /**
     * Rotate (change the heading) of the robot by the specified degrees. Note, we have had problems with overshoot on
     * turn, so this method method has an initial turn, and then a correct for turn error.
     *
     * @param degrees (double) The degrees of rotation, positive is clockwise (to the right).
     */
    protected void rotate(double degrees) {
        rotate(degrees, 1.0);
    }

    /**
     * Rotate (change the heading) of the robot by the specified degrees, at the specified maximum power. Note, we have had
     * problems with overshoot on turn, so this method method has an initial turn, and then a correct for turn error.
     *
     * @param degrees (double) The degrees of rotation, positive is clockwise (to the right).
     * @param maxPower (double) The maximum power for the rotation.
     */
    protected void rotate(double degrees, double maxPower) {
        rotate(degrees, maxPower, mtrAccelMin, mtrAccelDegs, mtrDecelMin, mtrDecelDegs);
    }

    /**
     * Rotate (change the heading) of the robot by the specified degrees, at the specified maximum power with the specified
     * tuning to the power ramp. Note, we have had problems with overshoot on turn, so this method method has an initial
     * turn, and then a correct for turn the overshoot.
     *
     * @param degrees (double) The degrees of rotation, positive is clockwise (to the right).
     * @param maxPower (double) The maximum power for the rotation.
     * @param accelMin (double) The power that should be applied when the initial rotation is 0.
     * @param accelDegs (double) The rotation (degrees) in which the power should increase from <tt>accelMin</tt> to 1.0.
     * @param decelMin (double) The power the motors should decelerate to immediately before the final rotation is reached.
     * @param decelDegs (double) The rotation (degrees) from the final rotation at which the power should start to decrease
     *                  from 1.0 towards <tt>decelMin</tt>.
     */
    protected void rotate(double degrees, double maxPower,
                          double accelMin, double accelDegs, double decelMin, double decelDegs) {
        expectedHeading = expectedHeading + degrees;
        recomputeHeading();
        // Rotate as specified
        turn(expectedHeading - heading, maxPower, accelMin, accelDegs, decelMin, decelDegs);
        // Test the heading and correct for error
        recomputeHeading();
        turn(expectedHeading - heading, maxPower, accelMin, accelDegs, decelMin, decelDegs);
    }

    /**
     * Turn the specified number of degrees at the specified maximum power with the specified
     * tuning to the power ramp. This stops when the target is reached.
     *
     * @param degrees (double) The degrees of rotation, positive is clockwise (to the right).
     * @param maxPower (double) The maximum power for the rotation.
     * @param accelMin (double) The power that should be applied when the initial rotation is 0.
     * @param accelDegs (double) The rotation (degrees) in which the power should increase from <tt>accelMin</tt> to 1.0.
     * @param decelMin (double) The power the motors should decelerate to immediately before the final rotation is reached.
     * @param decelDegs (double) The rotation (degrees) from the final rotation at which the power should start to decrease
     *                  from 1.0 towards <tt>decelMin</tt>.
     * @param decelDegs
     */
    private void turn(double degrees, double maxPower,
                      double accelMin, double accelDegs, double decelMin, double decelDegs) {
        resetDriveEncoders();
        double direction_mult = (degrees > 0) ? 1.0 : -1.0;
        double start_heading = heading;
        while (opModeIsActive() && direction_mult * (heading - start_heading) < degrees * direction_mult) {
            setDrive(0.0, 0.0, powerAccelDecel(direction_mult * (heading - start_heading), degrees * direction_mult,
                    maxPower, accelMin, accelDegs, decelMin, decelDegs) * direction_mult);
            recomputeHeading();
        }
        setDrive(0.0, 0.0, 0.0);
    }

}
