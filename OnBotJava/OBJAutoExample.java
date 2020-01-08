package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * A template for building autonomous OpModes. To create a new Op mode, make a copy of this with the name you want to use for
 * your op mode. Make sure you change the cl;ass name and the @Autonomous annotation name to match your file name.
 */
@Autonomous(name = "OBJAutoExample", group = "")
public class OBJAutoExample extends AMyMecBase {

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

        // memove this simple autonomous program and add your autonomous program here
        move(12.0, 45.0, 0.75);
        move(12.0, -45.0, 0.75);
        move(-12.0, 45.0, 0.75);
        move(-12.0, -45.0, 0.75);
        rotate(360.0, 0.6);
        rotate(-360.0, 0.6);
    }
}
