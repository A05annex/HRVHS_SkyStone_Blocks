package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The OnBotJava version of the blocks DriveExample.
 */
@TeleOp(name = "OBJDriveExample", group = "")
public class OBJDriveExample extends AMyMecBase {

    @Override
    protected void postStartInitialization() {
        super.postStartInitialization();
        // we don't know what the driver will do when driving starts. We start assuming heading may change and lock
        //  in the expected heading the first time there is no turn in the move commant.
        inTurn = true;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // initialize the control constants
        initConstants();
        // initialize the hardware before the opmode starts
        preStartInitialization();
        waitForStart();
        // the OpMode has started, do any initialization that needs to be postponed until the OpMode starts.
        postStartInitialization();
        // OK, the robot is ready to go
        // This is the main run loop - which is a simple drive loop
        while (opModeIsActive()) {
            // Gamepad X is the forced exit from the program.
            if (gamepad1.x) {
                break;
            }
            // This is the the real drive loop. In this loop we read/condition the control stick, then use the
            // conditioned values to set the drive speeds.
            conditionSticks();
            recomputeHeading();
            // Arcade drive - the right stick is direction (forward, backward, left, right) without changing the
            // heading of the robot.
            arcadeDrive();
            // Tank drive -
            // tankDrive();
            // Board-Relative Drive -
            // boardRelativeDrive();

            // Add your main control loop code here for connecting gamepad actions to your non-drive methods that
            // manipulate servos and motors.
        }
    }

    /**
     * This is tank drive for a driver sitting in the robot (i.e. the driver needs to imagine being a driver sitting in the
     * robot). The left stick controls left wheels forward and backward, the right stick controls right wheels forward and
     * backwards. Sideways motion is the average of the sideways values of the left and right sticks. Note that there is
     * no automatic heading correction in this drive mode.
     */
    protected void tankDrive() {
        // This first bit is that the maximum motor power is 1, BUT, the combination of sideways and forward/backward position
        // of the sticks could be greater than 1. In that case we need to scale the stick values so the movement is consistent
        // with the stick positions.
        double scale = 1.0;
        double aveX = 0.5 * (conditionedRightX + conditionedLeftX);
        double maxLeft = Math.abs(conditionedLeftY) + Math.abs(aveX);
        double maxRight = Math.abs(conditionedRightY) + Math.abs(aveX);
        if (maxLeft > 1.0) {
            scale = (maxRight > maxLeft) ? 1.0 / maxRight : 1.0 / maxLeft;
        } else if (maxRight > 1.0) {
            scale = 1.0 / maxRight;
        }
        // OK, we have a scale so none of the computed power values will exceed 1, compute and set the powers
        double powerRF = scale * (conditionedRightY - aveX);
        double powerRR = scale * (conditionedRightY + aveX);
        double powerLF = scale * (conditionedLeftY + aveX);
        double powerLR = scale * (conditionedLeftY - aveX);
        setPower(powerRF, powerRR, powerLF, powerLR);
        inTurn = true;
    }

    /**
     * This is arcade drive (similar to arcade game controls) for a driver sitting in the robot (i.e. the driver needs to
     * imagine being a driver sitting in the robot). In this drive mode the right stick controls robot speed/direction and the
     * left stick controls robot heading (speed/direction of rotation). Note that if only the right stick is being used (left
     * stick is withing the deadband) there is automatic heading correction using PID control driven by the heading reported
     * by the IMU to maintain the robot heading.
     */
    protected void arcadeDrive() {
        if (conditionedLeftX == 0.0) {
            // the left X (rotation) is within the deadband - the robot is not turning. If the robot was previously turning
            // then we reset the expected heading to the current heading and continue from there
            if (inTurn) {
                expectedHeading = heading;
                inTurn = false;
            }
            // get the heading error for use in the heading correction PID loop - we only us the P part here.
            double max = Math.abs(conditionedRightX) + Math.abs(conditionedRightY);
            double error = expectedHeading - heading;
            setDrive(conditionedRightY, conditionedRightX, max * kp * error);
        } else {
            setDrive(conditionedRightY, conditionedRightX, conditionedLeftX);
            inTurn = true;
        }
    }

    /**
     * This is a board-relative drive; that is, a drive mode relative to the driver standing by the field rather than a driver
     * sitting inside the robot
     */
    protected void boardRelativeDrive() {
        // TODO - find the code for this and put it in.
    }
}

