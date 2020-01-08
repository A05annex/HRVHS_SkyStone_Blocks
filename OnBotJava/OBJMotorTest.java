package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The OnBotJava version of the MotorTest
 */
@TeleOp(name = "OBJMotorTest", group = "")
public class OBJMotorTest extends AMyMecBase {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize the control constants
        initConstants();
        // initialize the hardware before the opmode starts
        preStartInitialization();
        waitForStart();
        // the OpMode has started, do any initialization that needs to be postponed until the OpMode starts.
        postStartInitialization();
        resetDriveEncoders();
        // OK, the robot is ready to go
        // This is the main run loop,we use the dpad positions to run each of the motors forward for testing
        while (opModeIsActive()) {
            if (gamepad1.x) {
                break;
            }
            double powerLF = gamepad1.dpad_up ? 1.0 : 0.0;
            double powerRF = gamepad1.dpad_right ? 1.0 : 0.0;
            double powerRR = gamepad1.dpad_down ? 1.0 : 0.0;;
            double powerLR = gamepad1.dpad_left ? 1.0 : 0.0;
            setPower(powerRF, powerRR, powerLF, powerLR);
            telemetry.addData("Front left speed", powerLF);
            telemetry.addData("Front left enc", FL.getCurrentPosition());
            telemetry.addData("Front right speed", powerRF);
            telemetry.addData("Front right enc", FR.getCurrentPosition());
            telemetry.addData("Rear right speed", powerRR);
            telemetry.addData("Rear right enc", RR.getCurrentPosition());
            telemetry.addData("Rear left speed", powerLR);
            telemetry.addData("Rear left enc", LR.getCurrentPosition());
            telemetry.update();
        }
    }
}
