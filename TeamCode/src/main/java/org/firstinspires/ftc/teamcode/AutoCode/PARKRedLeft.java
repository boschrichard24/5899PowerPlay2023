package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports


@Autonomous(name="PARKRedLeft Auto", group="CompetitionAuto")
//@Disabled
public class PARKRedLeft extends AutoSupplies {
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
            encoderMove(400, -1, 0);
            pause(800);
            encoderMove(861, 0, 0.6);
            pause(800);

        } else if (daWay == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            encoderMove(140, -0.35, 0.35);
            pause(800);
            turnToS(0,0.5,2);
            pause(800);
            encoderMove(700, 0, 0.85);
            pause(800);

        } else if(daWay == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            encoderMove(356, 1, 0);
            pause(800);
            encoderMove(861, 0, 0.6);
            pause(800);

        } else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            encoderMove(140, -0.35, 0.35);
            pause(800);
            turnToS(0,0.5,2);
            pause(800);
            encoderMove(700, 0, 0.85);
            pause(800);

        }

        if(daWay == 0) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
        }


    }
}