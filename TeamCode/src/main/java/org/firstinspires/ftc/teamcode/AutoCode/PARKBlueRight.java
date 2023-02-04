package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports

//                  O R I N G I N A L   A U T O   P A R K I N G                   \\


@Autonomous(name="PARKBlueRight Auto", group="CompetitionAuto")
//@Disabled
public class PARKBlueRight extends AutoSupplies {
    @Override
    public void runOpMode() {
        int daWay = 0;
        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        //waitForStart();
        initVision();
        //setArmEncoderMode();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition cone = null;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        long halfSec = 500;
        runtime.reset();
        while (runtime.milliseconds() <= halfSec) {
            cone = getConeType();
            if (cone != null) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                break;
            }
        }
        path = getZone(cone);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(300);
        if (path == 1) {
            daWay = 1;
        } else if (path == 2) {
            daWay = 2;
        } else if(path == 3){
            daWay = 3;
        } else{ lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }


        sleep(300);
        if (daWay == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            encoderMove(710, -0.5, 0);
            pause(1500);
            encoderMove(775, 0, 1);
        } else if (daWay == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            encoderMove(200, -0.5, 0.5);
            pause(1500);
            encoderMove(740, 0, 1);
        } else if(daWay == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            encoderMove(790, 0.5, 0);
            pause(1500);
            encoderMove(775, 0, 1);
        } else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            encoderMove(200, -0.5, 0.5);
            pause(1500);
            encoderMove(725, 0, 0.8);
        }

        if(daWay == 0) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
        }

    }
}